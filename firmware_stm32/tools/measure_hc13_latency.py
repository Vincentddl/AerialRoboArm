"""One-way HC13 application latency with TWO serial adapters on ONE host.

Measures before TX Serial.write -> RX read returning a complete matched frame.
Includes USB, drivers, UART serialization and radio; not isolated RF latency.
Uses non-control L-prefixed probes. No AT, V/VISION, servo or flash commands.
"""
from __future__ import annotations

import argparse
import binascii
import csv
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import secrets
import statistics
import sys
import threading
import time

import serial
from serial.tools import list_ports

ROOT=Path(__file__).resolve().parents[1]
BOUNDARY=('Same-host timestamp before TX Serial.write to RX Serial.read return '
          'containing the complete matched probe. Includes both USB adapters, '
          'OS/driver scheduling, UART and HC13 radio. Not isolated RF latency, '
          'MCU parser latency, or RTT/2.')


def packet(session,sequence):
    if not 0<=session<=0xffff or not 0<=sequence<=0xffff:
        raise ValueError('Session and sequence must fit four hex digits')
    body=f'L{session:04X}{sequence:04X}'.encode('ascii')
    return body+f'{binascii.crc_hqx(body,0xffff):04X}\n'.encode('ascii')


def parse_packet(raw):
    if len(raw)!=14 or raw[:1]!=b'L' or raw[-1:]!=b'\n':return None
    try:
        session=int(raw[1:5],16);sequence=int(raw[5:9],16);crc=int(raw[9:13],16)
    except ValueError:return None
    if raw!=packet(session,sequence):return None
    return session,sequence


def quantile(values,fraction):
    ordered=sorted(values);position=(len(ordered)-1)*fraction
    low=int(position);high=min(low+1,len(ordered)-1)
    return ordered[low]+(ordered[high]-ordered[low])*(position-low)


def latency_statistics(rows):
    values=[(r['rx_read_returned_ns']-r['tx_write_started_ns'])/1e6
            for r in rows if r.get('rx_read_returned_ns') is not None and not r.get('write_error')]
    if not values:return None
    return dict(count=len(values),mean_ms=statistics.mean(values),median_ms=statistics.median(values),
                p95_ms=quantile(values,.95),minimum_ms=min(values),maximum_ms=max(values))


class Tracker:
    def __init__(self,session):
        self.session=session;self.rows={};self.counts={};self.lock=threading.Lock()

    def count(self,key):self.counts[key]=self.counts.get(key,0)+1

    def register(self,sequence,stamp):
        with self.lock:
            self.rows[sequence]=dict(sequence=sequence,payload=packet(self.session,sequence).decode('ascii').rstrip(),
                                     tx_write_started_ns=stamp,tx_write_returned_ns=None,
                                     rx_read_returned_ns=None,write_error='')

    def observe(self,raw,stamp):
        with self.lock:
            parsed=parse_packet(raw)
            if parsed is None:self.count('invalid_frame');return
            session,sequence=parsed
            if session!=self.session or sequence not in self.rows:self.count('foreign_or_unissued');return
            row=self.rows[sequence]
            if stamp<row['tx_write_started_ns']:self.count('invalid_timestamp');return
            if row['rx_read_returned_ns'] is not None:self.count('duplicate');return
            row['rx_read_returned_ns']=stamp


def open_link(port,baud):
    link=serial.Serial(port=None,baudrate=baud,bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,
                       timeout=.02,write_timeout=.2,rtscts=False,dsrdtr=False)
    # Some HC-USB-T fixtures connect these lines to the AT configuration key.
    link.dtr=False;link.rts=False;link.port=port
    link.open();return link


def reader(link,tracker,stop,errors):
    pending=bytearray()
    try:
        while not stop.is_set():
            data=link.read(max(1,link.in_waiting))
            stamp=time.perf_counter_ns()  # Before parsing or logging.
            if not data:continue
            pending.extend(data)
            while b'\n' in pending:
                raw,_,rest=pending.partition(b'\n');pending[:]=rest
                tracker.observe(raw+b'\n',stamp)
            if len(pending)>4096:
                pending.clear()
                with tracker.lock:tracker.count('oversize_unterminated_input')
    except Exception as exc:errors.append(f'RX: {exc}');stop.set()


def save(folder,args,tracker,outcome,errors,ports):
    rows=list(tracker.rows.values())
    for row in rows:
        row['one_way_application_ms']=((row['rx_read_returned_ns']-row['tx_write_started_ns'])/1e6
                                        if row['rx_read_returned_ns'] is not None and not row['write_error'] else None)
    stats=latency_statistics(rows)
    flags=['Includes both USB adapters and OS scheduling; not isolated HC13 RF latency.',
           'Test probes match a 14-byte control-frame example, not its exact contents.',
           'Unreturned probes are measured only within this run and final drain window.',
           'Sparse 20 Hz probes do not characterize other traffic loads or distances.']
    if not stats or stats['count']<30:flags.append('Insufficient valid samples: fewer than 30.')
    if outcome!='duration_complete' or errors:flags.append('Incomplete or failed run; inspect errors.')
    result=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),boundary=BOUNDARY,
                method='two serial adapters, same host monotonic clock, session/sequence/CRC matching',
                outcome=outcome,run_completed=outcome=='duration_complete' and not errors,
                statistics=stats,requested_packets=args.count,sent_packets=len(rows),
                received_packets=sum(r['rx_read_returned_ns'] is not None for r in rows),
                unreturned_packets=sum(r['rx_read_returned_ns'] is None for r in rows),
                sufficient_sample_count=bool(stats and stats['count']>=30),counters=tracker.counts,
                arguments=vars(args),port_inventory=ports,errors=errors,quality_flags=flags,
                test_frame_bytes=14,ideal_uart_serialization_ms=14*10/args.baud*1000,
                actuator_commands_sent=False,at_commands_sent=False,firmware_modified=False,
                python=sys.version,pyserial=serial.__version__,
                script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    with (folder/'packets.csv').open('w',encoding='utf-8-sig',newline='') as handle:
        fields=list(rows[0]) if rows else ['sequence','one_way_application_ms']
        writer=csv.DictWriter(handle,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    print(json.dumps(result,ensure_ascii=False,indent=2));print(f'Saved: {folder}')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list',action='store_true',help='List devices without opening ports')
    parser.add_argument('--tx-port');parser.add_argument('--rx-port')
    parser.add_argument('--baud',type=int,default=230400)
    parser.add_argument('--count',type=int,default=200)
    parser.add_argument('--interval',type=float,default=.05)
    parser.add_argument('--drain-seconds',type=float,default=1)
    parser.add_argument('--output-dir',default=str(ROOT/'outputs/hc13_latency'))
    args=parser.parse_args(argv)
    ports=[dict(port=p.device,description=p.description,hwid=p.hwid) for p in list_ports.comports()]
    if args.list:
        print(json.dumps(ports,ensure_ascii=False,indent=2));return 0
    if not args.tx_port or not args.rx_port:parser.error('Two ports required: --tx-port COMx --rx-port COMy')
    if args.tx_port.upper()==args.rx_port.upper():parser.error('TX and RX must be different serial adapters')
    if not 1<=args.count<=5000 or not .01<=args.interval<=2 or not .1<=args.drain_seconds<=10 or args.baud<=0:
        parser.error('Use count 1..5000, interval .01..2 s, drain .1..10 s, positive baud')
    session=secrets.randbits(16);tracker=Tracker(session);stop=threading.Event();errors=[]
    folder=Path(args.output_dir)/f'{datetime.now():%Y%m%d_%H%M%S}_{session:04X}'
    folder.mkdir(parents=True,exist_ok=False)
    tx=rx=thread=None;outcome='error'
    try:
        # Open BOTH before transmitting; a missing receiver never triggers probes.
        rx=open_link(args.rx_port,args.baud);tx=open_link(args.tx_port,args.baud)
        time.sleep(.15);rx.reset_input_buffer()
        thread=threading.Thread(target=reader,args=(rx,tracker,stop,errors),daemon=True);thread.start()
        print(f'TX {args.tx_port} -> HC13 -> RX {args.rx_port}; {args.count} probes, no actuator commands.',flush=True)
        for sequence in range(1,args.count+1):
            if stop.is_set():raise RuntimeError('Receiver stopped')
            outgoing=packet(session,sequence)
            tracker.register(sequence,0)
            try:
                with tracker.lock:
                    stamp=time.perf_counter_ns()
                    tracker.rows[sequence]['tx_write_started_ns']=stamp
                n=tx.write(outgoing)
                returned=time.perf_counter_ns()
                with tracker.lock:
                    tracker.rows[sequence]['tx_write_returned_ns']=returned
                    if n!=14:tracker.rows[sequence]['write_error']=f'short_write:{n}'
                if n!=14:raise RuntimeError(f'Short UART write: {n}')
            except Exception as exc:
                with tracker.lock:tracker.rows[sequence]['write_error']=str(exc)
                raise
            remaining=args.interval-(time.perf_counter_ns()-stamp)/1e9
            if remaining>0:stop.wait(remaining)
        stop.wait(args.drain_seconds)
        outcome='duration_complete' if not errors else 'error'
    except KeyboardInterrupt:outcome='keyboard_interrupt'
    except Exception as exc:errors.append(str(exc));outcome='error'
    finally:
        stop.set()
        if thread is not None:
            thread.join(timeout=1)
            if thread.is_alive():errors.append('RX shutdown timeout')
        for link in (tx,rx):
            if link is not None:
                try:link.close()
                except Exception as exc:errors.append(f'close: {exc}')
        report=save(folder,args,tracker,outcome,errors,ports)
    return 0 if report['run_completed'] and report['sufficient_sample_count'] else 2


if __name__=='__main__':raise SystemExit(main())
