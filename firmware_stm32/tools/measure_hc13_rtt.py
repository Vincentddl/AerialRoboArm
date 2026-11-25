"""RTT against isolated HC13_RTT_ONLY firmware. No V/AT/motion commands.

The firmware uses an immediate diagnostic reply, not production task ACKs.
Measurement includes PC USB, forward wireless, MCU reply and reverse wireless.
"""
from __future__ import annotations
import argparse,binascii,csv,hashlib,json,secrets,sys,time
from datetime import datetime,timezone
from pathlib import Path
from measure_hc13_latency import open_link,packet,quantile


def parse_reply(raw):
    if len(raw)!=14 or raw[:1]!=b'R' or raw[-1:]!=b'\n':return None
    try:
        session=int(raw[1:5],16);sequence=int(raw[5:9],16)
        crc=int(raw[9:13],16)
    except ValueError:return None
    if binascii.crc_hqx(raw[:9],0xffff)!=crc:return None
    return session,sequence


def run(args):
    folder=Path(args.output_dir);folder.mkdir(parents=True,exist_ok=True)
    session=secrets.randbits(16);rows={};events=[];pending=bytearray();errors=[]
    link=None;outcome='error';misses=0
    try:
        link=open_link(args.port,args.baud);time.sleep(.15);link.reset_input_buffer()
        for sequence in range(1,args.count+args.warmup+1):
            outgoing=packet(session,sequence)
            row=dict(sequence=sequence,warmup=sequence<=args.warmup,payload=outgoing.decode().rstrip(),
                     tx_ns=None,write_returned_ns=None,rx_ns=None,rtt_ms=None,wait_expired=False)
            rows[sequence]=row
            row['tx_ns']=time.perf_counter_ns()
            n=link.write(outgoing);row['write_returned_ns']=time.perf_counter_ns()
            if n!=len(outgoing):raise RuntimeError(f'Short write {n}')
            deadline=time.perf_counter()+args.timeout
            while time.perf_counter()<deadline and row['rx_ns'] is None:
                chunk=link.read(max(1,link.in_waiting));stamp=time.perf_counter_ns()
                if not chunk:continue
                pending.extend(chunk)
                while b'\n' in pending:
                    raw,_,rest=pending.partition(b'\n');pending[:]=rest
                    parsed=parse_reply(raw+b'\n')
                    if parsed is None:
                        events.append(dict(kind='invalid_or_other_reply',line_hex=raw[:100].hex()));continue
                    sid,seq=parsed;previous=rows.get(seq)
                    if sid!=session or previous is None or stamp<previous['tx_ns']:
                        events.append(dict(kind='foreign_or_unissued_reply',sequence=seq));continue
                    if previous['rx_ns'] is not None:
                        events.append(dict(kind='duplicate_reply',sequence=seq));continue
                    previous['rx_ns']=stamp;previous['rtt_ms']=(stamp-previous['tx_ns'])/1e6
                if len(pending)>4096:pending.clear();events.append(dict(kind='oversize_input'))
            if row['rx_ns'] is None:row['wait_expired']=True;misses+=1
            else:misses=0
            if misses>=3:raise RuntimeError('Three consecutive probes without diagnostic R reply')
            if sequence%25==0:print(f'Completed {sequence}/{args.count+args.warmup}',flush=True)
            remaining=args.interval-(time.perf_counter_ns()-row['tx_ns'])/1e9
            if remaining>0:time.sleep(remaining)
        outcome='duration_complete'
    except KeyboardInterrupt:outcome='keyboard_interrupt'
    except Exception as exc:errors.append(str(exc))
    finally:
        if link is not None:link.close()
        records=list(rows.values());valid=[r['rtt_ms'] for r in records if not r['warmup'] and r['rtt_ms'] is not None]
        stats=(dict(count=len(valid),mean_ms=sum(valid)/len(valid),median_ms=quantile(valid,.5),p95_ms=quantile(valid,.95),minimum_ms=min(valid),maximum_ms=max(valid)) if valid else None)
        result=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),session=session,
                    metric='PC_write_to_diagnostic_MCU_reply_read_RTT_ms',
                    boundary='Before host serial.write to read returning complete CRC-valid matching R reply; includes PC USB, both radio directions, MCU immediate response and UART.',
                    production_firmware=False,production_20ms_task_included=False,production_200ms_ACK_gate_included=False,
                    one_way_latency_measured=False,motion_commands_sent=False,request_bytes=14,reply_bytes=14,
                    statistics=stats,outcome=outcome,run_completed=outcome=='duration_complete',
                    requested_samples=args.count,warmup_samples=args.warmup,
                    nonwarmup_unreturned=sum(not r['warmup'] and r['rx_ns'] is None for r in records),
                    arguments=vars(args),errors=errors,
                    cautions=['RTT is not one-way latency; do not divide by two as a direct measurement.',
                              'Immediate diagnostic response differs from production task/parser timing.',
                              'Only received samples enter delay quantiles; inspect unreturned probes.',
                              'Distance, mode and RF conditions must be recorded separately.'],
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        with (folder/'packets.csv').open('w',encoding='utf-8-sig',newline='') as h:
            w=csv.DictWriter(h,fieldnames=list(records[0]) if records else ['sequence','rtt_ms']);w.writeheader();w.writerows(records)
        (folder/'events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    return 0 if outcome=='duration_complete' and stats and stats['count']>=30 else 2


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=230400)
    p.add_argument('--count',type=int,default=200);p.add_argument('--warmup',type=int,default=5)
    p.add_argument('--interval',type=float,default=.05);p.add_argument('--timeout',type=float,default=1)
    p.add_argument('--output-dir',required=True)
    a=p.parse_args()
    if not 1<=a.count<=5000 or not 0<=a.warmup<=30 or not .01<=a.interval<=2 or not .1<=a.timeout<=5:p.error('Invalid count, interval or timeout')
    raise SystemExit(run(a))
