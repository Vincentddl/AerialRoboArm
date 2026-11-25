"""Record HX8 moves and JY60 responses using MCU timestamps over RTT channel 1.

Default: monitor successive VOFA commands; save moves only when MCU recording is enabled.
--once records one passive move; --target explicitly sends one move on console 0.
"""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import socket
import sys
import time

VALUE = re.compile(r'\b([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)')
FIELDS = ['stream','seq','t_mcu_ms','t_relative_ms','x','y','z','unit',
          'angle_deg','voltage_mv','current_ma','status','tail_raw']


def parse_line(line):
    if line.startswith('[I] '):
        try:
            values=[int(v) for v in line[4:].split(',')]
        except ValueError:
            return None
        if len(values)!=7 or values[2] not in (81,82,83):return None
        return dict(zip(('seq','t_ms','type','x','y','z','tail'),values),stream='IMU',event='',raw=line)
    match = re.match(r'^\[(IMU|HX8|CAP)\]\s*(.*)', line.strip())
    if not match:
        return None
    result = {'stream': match[1], 'event': match[2].split()[0] if match[2] else '', 'raw': line}
    for key, value in VALUE.findall(match[2]):
        try:
            result[key] = int(value,16) if value.startswith('0x') else int(value)
        except ValueError:
            result[key] = value
    return result


def relative_ms(timestamp, start):
    delta = (timestamp-start) & 0xFFFFFFFF
    return delta if delta < 0x80000000 else delta-0x100000000


def sample_row(record, start):
    row = {'stream':record['stream'], 'seq':record.get('seq'),
           't_mcu_ms':record['t_ms'], 't_relative_ms':relative_ms(record['t_ms'],start)}
    if record['stream']=='IMU':
        kind=record.get('type'); row['stream']={81:'ACC',82:'GYRO',83:'ANGLE'}.get(kind,'IMU')
        row['unit']={81:'g',82:'deg/s',83:'deg'}.get(kind,'unknown')
        for axis in ('x','y','z'):
            row[axis]=record[axis]/1000.0
        row['tail_raw']=record.get('tail')
    else:
        row.update(angle_deg=record['angle_mdeg']/1000.0,voltage_mv=record.get('voltage'),
                   current_ma=record.get('current'),status=record.get('status'))
    return row


def _record_once(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=9091)
    parser.add_argument('--console-port',type=int,default=9090)
    parser.add_argument('--target',type=int,help='Explicitly command HX8 angle; omit to listen for VOFA.')
    parser.add_argument('--speed',type=int,default=500)
    parser.add_argument('--ramp-ms',type=int,default=100)
    parser.add_argument('--post-ms',type=int,default=1000)
    parser.add_argument('--pre-ms',type=int,default=200)
    parser.add_argument('--wait-seconds',type=float,default=120)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    if args.target is not None and not -90<=args.target<=85:
        parser.error('target must be -90..85 degrees')
    if not (1<=args.speed<=500 and 20<=args.ramp_ms<=2000 and 0<=args.post_ms<=5000 and 0<=args.pre_ms<=2000):
        parser.error('invalid speed/ramp/post/pre setting')
    out=args.output or Path(__file__).resolve().parents[1]/'outputs'/'hx8_jy60_capture'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True,exist_ok=False)
    baseline=deque(maxlen=2000); start=None; finish=None; rows=0; gaps={'IMU':0,'HX8':0}; last={}
    command_sent=False; arm_requested=False; capture_started_host=None; error=None
    primed=False
    summary={'host_started_at':datetime.now().astimezone().isoformat(),'parameters':vars(args)|{'output':str(out)}}
    summary['parameters']['output']=str(out)
    deadline=time.monotonic()+args.wait_seconds
    print('Listening on RTT 9091. In VOFA 9090 send g <angle> when ready.',flush=True)
    print('OUTPUT:',out,flush=True)
    console=None
    try:
        with socket.create_connection((args.host,args.port),timeout=3) as link, \
             (out/'raw.jsonl').open('w',encoding='utf-8') as raw, \
             (out/'samples.csv').open('w',newline='',encoding='utf-8-sig') as data:
            link.settimeout(.5); writer=csv.DictWriter(data,fieldnames=FIELDS);writer.writeheader();pending=b''
            for_stop=False
            while not for_stop:
                if time.monotonic()>deadline:
                    raise RuntimeError('Timed out waiting for command or END event; recording is incomplete.')
                try:
                    chunk=link.recv(16384)
                except socket.timeout:
                    continue
                if not chunk:raise RuntimeError('RTT connection closed; recording is incomplete.')
                pending+=chunk
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1)
                    record=parse_line(line.decode('utf-8',errors='replace').replace('\x00','').strip())
                    if not record:continue
                    record['host_received_at']=datetime.now().astimezone().isoformat()
                    raw.write(json.dumps(record,ensure_ascii=False)+'\n')
                    if record['stream']=='CAP' and record['event']=='BOOT' and start is not None:
                        raise RuntimeError('MCU restarted during capture.')
                    capture_mode=record.get('mode')!='MANUAL_HX8' and record.get('record',1)!=0
                    if record['stream']=='CAP' and record['event']=='READY' and capture_mode and record.get('ready')==1 and len(baseline)>=12:
                        if not primed:print('READY: send one g <angle> in VOFA now.',flush=True)
                        primed=True
                    if args.target is not None and not command_sent and record['stream']=='CAP' and record['event']=='READY':
                        if not capture_mode:
                            if not arm_requested and record.get('phase',0)==0:
                                console=socket.create_connection((args.host,args.console_port),timeout=3)
                                console.sendall(b'\nr 1\n');arm_requested=True
                            continue
                        if record.get('ready')!=1:continue
                        # Baseline frames already have reached the PC; begin command only after warm-up.
                        if len(baseline)<12:continue
                        if console is None:console=socket.create_connection((args.host,args.console_port),timeout=3)
                        command=f'\nv {args.speed}\na {args.ramp_ms}\nw {args.post_ms}\ng {args.target}\n'
                        console.sendall(command.encode('ascii'));command_sent=True
                    if record['stream']=='CAP' and record['event']=='REJECT' and command_sent and start is None:
                        raise RuntimeError('MCU rejected command: '+record['raw'])
                    if record['stream'] in ('IMU','HX8'):
                        if start is None:baseline.append(record);continue
                        row=sample_row(record,start['t_ms']);writer.writerow(row);rows+=1
                        stream=record['stream'];seq=record.get('seq')
                        if stream in last:gaps[stream]+=max(0,seq-last[stream]-1)
                        last[stream]=seq
                    elif record['stream']=='CAP' and record['event']=='START':
                        if not primed:
                            # Ignore an old event buffered before we saw a live idle state.
                            continue
                        if start is not None:raise RuntimeError('Another command started before recording completed.')
                        start=record;capture_started_host=time.monotonic();deadline=capture_started_host+20
                        for old in baseline:
                            if -args.pre_ms<=relative_ms(old['t_ms'],start['t_ms'])<=0:
                                writer.writerow(sample_row(old,start['t_ms']));rows+=1
                        last={'IMU':start.get('imu_seq',0),'HX8':start.get('hx_seq',0)}
                        print('CAPTURE START: MCU time',start['t_ms'],'target',start.get('target_mdeg',0)/1000.0,flush=True)
                    elif record['stream']=='CAP' and record['event']=='ARRIVED':
                        print('HX8 arrived; recording post-motion response.',flush=True)
                    elif record['stream']=='CAP' and record['event']=='END' and start is not None:
                        if record.get('id')!=start.get('id'):raise RuntimeError('Command id mismatch.')
                        finish=record;for_stop=True;break
    except (OSError,RuntimeError,KeyboardInterrupt) as exc:
        error=str(exc) or 'Interrupted by operator'
        if start is not None and finish is None:
            try:
                with socket.create_connection((args.host,args.console_port),timeout=2) as stop_link:
                    stop_link.sendall(b'\nx\n')
            except OSError:pass
    finally:
        if console:console.close()
        summary.update(start=start,end=finish,sample_rows=rows,sequence_gaps=gaps,error=error)
        summary['complete']=bool(finish and finish.get('reason')=='complete' and not error and not finish.get('drops') and not finish.get('imu_errors') and not finish.get('uart_recoveries') and not any(gaps.values()))
        (out/'metadata.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,default=str),encoding='utf-8')
    print('SAVED:',out,'COMPLETE:',summary['complete'],flush=True)
    if error:print('ERROR:',error,flush=True)
    return 0 if summary['complete'] else 1


class ContinuousTrial:
    def __init__(self, folder, start, baseline, pre_ms):
        self.folder=folder;folder.mkdir()
        self.start=start;self.arrived=None;self.rows=0
        self.request=None
        self.gaps={'IMU':0,'HX8':0}
        self.last={'IMU':start.get('imu_seq',0),'HX8':start.get('hx_seq',0)}
        self.raw=(folder/'raw.jsonl').open('w',encoding='utf-8')
        self.data=(folder/'samples.csv').open('w',newline='',encoding='utf-8-sig')
        self.writer=csv.DictWriter(self.data,fieldnames=FIELDS);self.writer.writeheader()
        for record in baseline:
            if -pre_ms<=relative_ms(record['t_ms'],start['t_ms'])<=0:
                self.writer.writerow(sample_row(record,start['t_ms']));self.rows+=1;self.log(record)
        self.log(start)

    def log(self,record):
        self.raw.write(json.dumps(record,ensure_ascii=False)+'\n')

    def sample(self,record):
        self.log(record);self.writer.writerow(sample_row(record,self.start['t_ms']));self.rows+=1
        stream=record['stream'];seq=record['seq']
        self.gaps[stream]+=max(0,seq-self.last[stream]-1)
        if seq<=self.last[stream]:raise RuntimeError('Sample sequence reset or duplicated.')
        self.last[stream]=seq

    def close(self,end=None,error=None):
        if end:
            self.log(end)
            for stream,key in [('IMU','imu_seq'),('HX8','hx_seq')]:
                if key in end:self.gaps[stream]+=max(0,end[key]-self.last[stream])
        result={'request':self.request,'accepted':True,'start':self.start,'arrived':self.arrived,'end':end,'sample_rows':self.rows,
                'sequence_gaps':self.gaps,'error':error}
        result['complete']=bool(end and end.get('reason')=='complete' and not error and
                                not end.get('drops') and not end.get('imu_errors') and not end.get('uart_recoveries') and not any(self.gaps.values()))
        self.raw.close();self.data.close()
        (self.folder/'metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        return result


def _record_continuous(argv):
    parser=argparse.ArgumentParser(description='Continuously capture each VOFA HX8 move into its own folder. No movement commands are sent.')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=9091)
    parser.add_argument('--console-port',type=int,default=9090)
    parser.add_argument('--pre-ms',type=int,default=200)
    parser.add_argument('--wait-seconds',type=float,default=0,help='Idle timeout; 0 waits indefinitely.')
    parser.add_argument('--stream-timeout',type=float,default=5,help='Stop visibly if no valid telemetry arrives for this many seconds.')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    if not 0<=args.pre_ms<=2000 or args.wait_seconds<0 or args.stream_timeout<=0:parser.error('invalid pre-ms/wait-seconds/stream-timeout')
    out=args.output or Path(__file__).resolve().parents[1]/'outputs/hx8_jy60_capture'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True,exist_ok=False)
    baseline=deque(maxlen=2000);active=None;primed=False;trials=[];error=None;requests={}
    last_activity=time.monotonic();deadline=None
    last_telemetry=last_activity;last_display=0;last_drops=None;imu_rate={}
    print('CAPTURE MONITOR: r 1 enables recording; r 0 disables it. The MCU mode decides whether each g move is saved.',flush=True)
    print('OUTPUT:',out,flush=True)
    try:
        with socket.create_connection((args.host,args.port),timeout=3) as link, (out/'session_events.jsonl').open('w',encoding='utf-8') as events:
            link.settimeout(.5);pending=b''
            while True:
                now=time.monotonic()
                if now-last_telemetry>args.stream_timeout:
                    raise RuntimeError('STREAM STALE: no valid MCU telemetry for %.1f s. STOP TESTING; reconnect RTT and restart capture.' % args.stream_timeout)
                if active and deadline is not None and now>deadline:
                    raise RuntimeError('Missing END event; active recording is incomplete.')
                if not active and args.wait_seconds and now-last_activity>args.wait_seconds:break
                try:chunk=link.recv(16384)
                except socket.timeout:continue
                if not chunk:raise RuntimeError('RTT connection closed.')
                pending+=chunk
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1)
                    record=parse_line(line.decode('utf-8',errors='replace').replace('\x00','').strip())
                    if not record:continue
                    last_telemetry=time.monotonic()
                    record['host_received_at']=datetime.now().astimezone().isoformat()
                    if record['stream'] in ('IMU','HX8'):
                        if active:active.sample(record)
                        baseline.append(record)
                        continue
                    events.write(json.dumps(record,ensure_ascii=False)+'\n');events.flush()
                    if record['event']=='IMU_RATE':imu_rate=record
                    if record['event']=='READY' and time.monotonic()-last_display>=2:
                        total=record.get('drops_total',0)
                        delta=0 if last_drops is None else max(0,total-last_drops)
                        last_drops=total;last_display=time.monotonic()
                        enabled=bool(record.get('record',int(record.get('mode')!='MANUAL_HX8')))
                        print('LIVE',datetime.now().strftime('%H:%M:%S'),
                              'READY='+str(record.get('ready',0)),
                              'HX8='+str(record.get('hx8','?')),'IMU='+str(record.get('imu','?')),
                              'Hz='+','.join(str(imu_rate.get(k,'?')) for k in ('acc_hz','gyro_hz','angle_hz')),
                              'NEW_DROPS='+str(delta),'RECORD='+('ON' if enabled else 'OFF'),
                              'RECORDING' if active else 'WAITING_FOR_G' if enabled else 'VIEW_ONLY',flush=True)
                    if record['event']=='BOOT':
                        if active:raise RuntimeError('MCU restarted during recording.')
                        baseline.clear();primed=False;requests.clear()
                    elif record['event']=='REQUEST':
                        requests[record['request_id']]=record
                    elif record['event']=='READY' and record.get('mode')!='MANUAL_HX8' and record.get('record',1)!=0 and record.get('ready')==1 and len(baseline)>=12:
                        if not primed:print('READY: send g <angle> in VOFA. Subsequent moves are recorded automatically.',flush=True)
                        primed=True
                    elif record['event']=='START':
                        if not primed and record.get('request_id') not in requests:continue
                        if active:raise RuntimeError('Another START before previous END.')
                        tag=str(record.get('target_mdeg',0)//1000).replace('-','neg')
                        folder=out/f'move_{len(trials)+1:03d}_cmd{record.get("id",0)}_g{tag}'
                        active=ContinuousTrial(folder,record,baseline,args.pre_ms)
                        active.request=requests.pop(record.get('request_id'),None)
                        if active.request:active.log(active.request)
                        deadline=time.monotonic()+20;last_activity=time.monotonic()
                        print('RECORDING:',folder.name,flush=True)
                    elif record['event']=='ARRIVED' and active:
                        active.arrived=record;active.log(record)
                    elif record['event']=='END' and active:
                        if record.get('id')!=active.start.get('id'):raise RuntimeError('Command id mismatch.')
                        folder=active.folder;result=active.close(record);active=None;deadline=None
                        trials.append({'folder':folder.name,'complete':result['complete'],'reason':record.get('reason')})
                        last_activity=time.monotonic()
                        print('SAVED:',folder.name,'COMPLETE:',result['complete'],'REASON:',record.get('reason'),flush=True)
                        print('  ROWS:',result['sample_rows'],'DROPS:',record.get('drops',0),
                              'GAPS:',result['sequence_gaps'],'CHECKSUM_ERRORS:',record.get('imu_errors',0),flush=True)
                        print('  FILE:',folder/'samples.csv',flush=True)
                        print('WAITING for next VOFA command; capture stays active.',flush=True)
                    elif record['event']=='REJECT':
                        request=requests.pop(record.get('request_id'),None)
                        if request:
                            tag=str(request.get('target_mdeg',0)//1000).replace('-','neg')
                            folder=out/f'request_{len(trials)+1:03d}_req{request["request_id"]}_g{tag}_rejected'
                            folder.mkdir()
                            result={'request':request,'accepted':False,'start':None,'end':None,
                                    'rejection':record,'complete':False,'sample_rows':0}
                            (folder/'metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
                            (folder/'raw.jsonl').write_text(json.dumps(request)+'\n'+json.dumps(record)+'\n',encoding='utf-8')
                            trials.append({'folder':folder.name,'accepted':False,'complete':False,'reason':record.get('reason')})
                        print('MCU rejected command:',record.get('reason'),flush=True)
                    elif record['event'] in ('MODE','RECORD_SWITCH','MANUAL_START','MANUAL_ARRIVED','MANUAL_END','MANUAL_REJECT','RECOVERY_START','RECOVERY_ARRIVED','RECOVERY_END','RECOVERY_REJECT'):
                        if record['event']=='MODE':
                            primed=False;baseline.clear()
                        print('MCU',record.get('raw'),flush=True)
    except KeyboardInterrupt:
        if active:error='Interrupted during movement recording'
    except (OSError,RuntimeError) as exc:
        error=str(exc)
    finally:
        if active:
            try:
                with socket.create_connection((args.host,args.console_port),timeout=2) as stop_link:stop_link.sendall(b'\nx\n')
            except OSError:pass
            folder=active.folder;active.close(error=error or 'recording interrupted')
            trials.append({'folder':folder.name,'complete':False,'reason':error})
        summary={'continuous':True,'trials':trials,'error':error,'output':str(out)}
        (out/'session_metadata.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    if error:print('STOPPED:',error,flush=True)
    return 1 if error or any(not r['complete'] for r in trials) else 0


def main(argv=None):
    argv=list(sys.argv[1:] if argv is None else argv)
    if '--once' in argv or any(a=='--target' or a.startswith('--target=') for a in argv):
        return _record_once([a for a in argv if a!='--once'])
    return _record_continuous(argv)


if __name__=='__main__':raise SystemExit(main())
