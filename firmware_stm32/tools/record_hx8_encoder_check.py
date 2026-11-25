"""Read-only RTT1 logger for a user-operated encoder motion check."""
import argparse,json,socket,time
from datetime import datetime
from pathlib import Path
from capture_hx8_jy60 import parse_line

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=300)
    args=parser.parse_args()
    folder=Path(__file__).resolve().parents[1]/'outputs/hx8_encoder_check'/datetime.now().strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True,exist_ok=False)
    print('READ ONLY; no servo commands are sent. OUTPUT:',folder,flush=True)
    count=0;error=None;started=time.monotonic();last_print=0;last_data=started
    try:
        with socket.create_connection(('127.0.0.1',9091),timeout=3) as link,(folder/'feedback.jsonl').open('w',encoding='utf-8') as output:
            link.settimeout(.5);pending=b''
            while time.monotonic()-started<args.seconds:
                if time.monotonic()-last_data>5:raise RuntimeError('No valid telemetry for 5 seconds.')
                try:chunk=link.recv(16384)
                except socket.timeout:continue
                if not chunk:raise RuntimeError('RTT connection closed.')
                pending+=chunk
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1)
                    record=parse_line(line.decode(errors='replace').replace('\x00','').strip())
                    if not record:continue
                    last_data=time.monotonic()
                    if record['stream']=='HX8' or (record['stream']=='CAP' and record.get('event') not in ('IMU_RATE','READY')):
                        record['host_received_at']=datetime.now().astimezone().isoformat()
                        output.write(json.dumps(record,ensure_ascii=False)+'\n');output.flush()
                    if record['stream']=='HX8':
                        count+=1
                        if time.monotonic()-last_print>=1:
                            last_print=time.monotonic();print('ANGLE:',record['angle_mdeg']/1000,'deg; samples:',count,flush=True)
    except (OSError,RuntimeError,KeyboardInterrupt) as exc:
        error=str(exc) or 'Interrupted'
    (folder/'summary.json').write_text(json.dumps({'samples':count,'error':error,'read_only':True},indent=2),encoding='utf-8')
    print('STOPPED:',error or 'time limit reached',flush=True)
    return 1 if error else 0

if __name__=='__main__':raise SystemExit(main())
