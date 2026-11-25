import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SCRIPT=Path(__file__).resolve().parents[1]/'tools/capture_hx8_jy60.py'
spec=importlib.util.spec_from_file_location('capture',SCRIPT)
capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)


class Link:
    def __init__(self,lines):
        raw=('\n'.join(lines)+'\n').encode();self.parts=[raw[:37],raw[37:123],raw[123:]];self.sent=[]
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def settimeout(self,*args):pass
    def recv(self,*args):return self.parts.pop(0) if self.parts else b''
    def sendall(self,data):self.sent.append(data)
    def close(self):pass


class CaptureTests(unittest.TestCase):
    def run_capture(self,missing=False,imu_errors=0,target=False,reason='complete',boot_off=False):
        lines=[f'[IMU] seq={i+1} t_ms={800+i*15} type=83 x=-1000 y=2000 z=0 tail=0' for i in range(12)]
        if boot_off:lines+=['[CAP] READY t_ms=980 mode=MANUAL_HX8 record=0 ready=1 phase=0',
                            '[CAP] MODE mode=CAPTURE record=1 imu_required=1']
        lines += ['[CAP] READY t_ms=990 ready=1 phase=0 hx8=1 imu=1',
                  '[CAP] START id=1 t_ms=1000 tx_end_ms=1002 target_mdeg=-20000 speed=220 acc_ms=100 dec_ms=100 post_ms=1000 imu_seq=12 hx_seq=0',
                  f'[IMU] seq={14 if missing else 13} t_ms=1004 type=82 x=-1200 y=0 z=0 tail=1',
                  '[HX8] seq=1 t_ms=1050 angle_mdeg=-20000 voltage=12000 current=20 status=0x01',
                  '[CAP] ARRIVED id=1 t_ms=1300 angle_mdeg=-20000',
                  f'[CAP] END id=1 t_ms=2300 reason={reason} drops=0 imu_errors={imu_errors} imu_seq=13 hx_seq=1']
        data_link=Link(lines);console=Link([])
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'round';args=['--once','--output',str(out)]
            if target:args+=['--target','-20']
            def connect(address,**kwargs):return data_link if address[1]==9091 else console
            with patch.object(capture.socket,'create_connection',side_effect=connect),contextlib.redirect_stdout(io.StringIO()):
                result=capture.main(args)
            meta=json.loads((out/'metadata.json').read_text())
            with (out/'samples.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
            return result,meta,rows,console.sent

    def test_vofa_passive_capture(self):
        result,meta,rows,sent=self.run_capture()
        self.assertEqual(result,0);self.assertTrue(meta['complete']);self.assertFalse(sent)
        self.assertEqual(rows[0]['t_relative_ms'],'-200')
        gyro=next(r for r in rows if r['stream']=='GYRO')
        self.assertEqual(gyro['t_relative_ms'],'4');self.assertEqual(gyro['x'],'-1.2')

    def test_missing_sample_invalidates_record(self):
        result,meta,_,_=self.run_capture(missing=True)
        self.assertEqual(result,1);self.assertEqual(meta['sequence_gaps']['IMU'],1)

    def test_sensor_checksum_errors_invalidate_record(self):
        result,meta,_,_=self.run_capture(imu_errors=1)
        self.assertEqual(result,1);self.assertFalse(meta['complete'])

    def test_fault_ends_without_claiming_success(self):
        result,meta,_,_=self.run_capture(reason='hx8_fault')
        self.assertEqual(result,1);self.assertFalse(meta['complete'])

    def test_explicit_command_mode(self):
        result,meta,_,sent=self.run_capture(target=True)
        self.assertEqual(result,0);self.assertIn(b'g -20\n',b''.join(sent))

    def test_explicit_target_arms_before_sending_angle(self):
        result,meta,_,sent=self.run_capture(target=True,boot_off=True)
        self.assertEqual(result,0);self.assertTrue(meta['complete'])
        self.assertEqual(sent[0],b'\nr 1\n')
        self.assertNotIn(b'g ',sent[0]);self.assertIn(b'g -20\n',sent[1])

    def test_mcu_clock_wrap(self):
        self.assertEqual(capture.relative_ms(0x10,0xFFFFFFF0),32)
        self.assertEqual(capture.relative_ms(0xFFFFFFF0,0x10),-32)

    def continuous_rounds(self,second_reason='complete',second_target=-60000):
        lines=[f'[IMU] seq={i+1} t_ms={800+i*15} type=83 x=0 y=0 z=0 tail=0' for i in range(12)]
        lines+=['[CAP] READY t_ms=990 ready=1',
                '[CAP] START id=1 t_ms=1000 target_mdeg=-20000 imu_seq=12 hx_seq=0',
                '[IMU] seq=13 t_ms=1010 type=82 x=1000 y=0 z=0 tail=0',
                '[HX8] seq=1 t_ms=1050 angle_mdeg=-20000 voltage=12000 current=14 status=0x00',
                '[CAP] ARRIVED id=1 t_ms=1300',
                '[CAP] END id=1 t_ms=2300 reason=complete drops=0 imu_errors=0 imu_seq=13 hx_seq=1',
                f'[CAP] START id=2 t_ms=2500 target_mdeg={second_target} imu_seq=13 hx_seq=1',
                '[IMU] seq=14 t_ms=2510 type=82 x=2000 y=0 z=0 tail=0',
                '[HX8] seq=2 t_ms=2550 angle_mdeg=-58600 voltage=12000 current=14 status=0x00',
                f'[CAP] END id=2 t_ms=4500 reason={second_reason} drops=0 imu_errors=0 imu_seq=14 hx_seq=2']
        class ContinuousLink(Link):
            def recv(self,*args):
                if not self.parts:raise KeyboardInterrupt()
                return self.parts.pop(0)
        link=ContinuousLink(lines)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'session'
            with patch.object(capture.socket,'create_connection',return_value=link),contextlib.redirect_stdout(io.StringIO()):
                code=capture.main(['--output',str(out)])
            session=json.loads((out/'session_metadata.json').read_text())
            metas=[json.loads(p.read_text()) for p in sorted(out.glob('move_*/metadata.json'))]
            return code,session,metas,link.sent

    def test_continuous_records_second_move_without_reconnect(self):
        code,session,metas,sent=self.continuous_rounds()
        self.assertEqual(code,0);self.assertEqual(len(metas),2);self.assertFalse(sent)
        self.assertEqual([m['start']['target_mdeg'] for m in metas],[-20000,-60000])
        self.assertTrue(all(m['complete'] for m in metas));self.assertIsNone(session['error'])

    def test_continuous_preserves_failed_second_round(self):
        code,_,metas,_=self.continuous_rounds('motion_timeout')
        self.assertEqual(code,1);self.assertTrue(metas[0]['complete'])
        self.assertFalse(metas[1]['complete']);self.assertGreater(metas[1]['sample_rows'],0)

    def test_repeated_same_target_is_separate_recording(self):
        code,_,metas,_=self.continuous_rounds(second_target=-20000)
        self.assertEqual(code,0);self.assertEqual(len(metas),2)
        self.assertEqual([m['start']['target_mdeg'] for m in metas],[-20000,-20000])

    def test_jy61p_200hz_preserves_all_three_streams(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial=capture.ContinuousTrial(Path(tmp)/'trial',{'t_ms':1000,'imu_seq':0,'hx_seq':0},[],200)
            seq=0
            for tick in range(400):
                for kind in (81,82,83):
                    seq+=1
                    trial.sample(capture.parse_line(f'[I] {seq},{1000+tick*5},{kind},1000,-2000,0,-30263'))
            meta=trial.close({'reason':'complete','imu_seq':1200,'hx_seq':0,'drops':0,'imu_errors':0,'uart_recoveries':0})
            self.assertTrue(meta['complete']);self.assertEqual(meta['sample_rows'],1200)
            with (Path(tmp)/'trial'/'samples.csv').open(encoding='utf-8-sig') as data:
                rows=list(csv.DictReader(data))
            for stream in ('ACC','GYRO','ANGLE'):
                values=[r for r in rows if r['stream']==stream]
                self.assertEqual(len(values),400)
                self.assertEqual(int(values[1]['t_relative_ms'])-int(values[0]['t_relative_ms']),5)
                self.assertEqual(values[0]['x'],'1.0')
                self.assertEqual(values[0]['y'],'-2.0')
                self.assertEqual(values[0]['tail_raw'],'-30263')

    def test_uart_recovery_invalidates_record(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial=capture.ContinuousTrial(Path(tmp)/'trial',{'t_ms':1000},[],200)
            meta=trial.close({'reason':'complete','drops':0,'imu_errors':0,'uart_recoveries':1})
            self.assertFalse(meta['complete'])

    def test_stalled_open_socket_reports_failure(self):
        class SilentLink(Link):
            def recv(self,*args):raise capture.socket.timeout()
        clock=[0]
        def tick():
            clock[0]+=1
            return clock[0]
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'session';link=SilentLink([]);output=io.StringIO()
            with patch.object(capture.socket,'create_connection',return_value=link),patch.object(capture.time,'monotonic',side_effect=tick),contextlib.redirect_stdout(output):
                code=capture.main(['--output',str(out),'--stream-timeout','5'])
            self.assertEqual(code,1)
            self.assertIn('STREAM STALE',json.loads((out/'session_metadata.json').read_text())['error'])
            self.assertIn('STOPPED:',output.getvalue())
            self.assertFalse(link.sent)

    def test_manual_hx8_events_do_not_claim_an_imu_recording(self):
        class ManualLink(Link):
            def recv(self,*args):
                if not self.parts:raise KeyboardInterrupt()
                return self.parts.pop(0)
        lines=[f'[HX8] seq={i+1} t_ms={800+i*50} angle_mdeg=0 voltage=12000 current=14 status=0x00' for i in range(12)]
        lines += ['[CAP] MODE mode=MANUAL_HX8 imu_required=0',
                  '[CAP] READY t_ms=1400 mode=MANUAL_HX8 ready=1 phase=0 hx8=1 imu=0',
                  '[CAP] MANUAL_START id=1 t_ms=1500 target_mdeg=-20000 speed=500 acc_ms=100 dec_ms=100 record=OFF',
                  '[CAP] MANUAL_END id=1 t_ms=2800 reason=complete angle_mdeg=-19800']
        link=ManualLink(lines)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'session';output=io.StringIO()
            with patch.object(capture.socket,'create_connection',return_value=link),contextlib.redirect_stdout(output):
                code=capture.main(['--output',str(out)])
            self.assertEqual(code,0)
            self.assertFalse(list(out.glob('move_*')))
            self.assertEqual(json.loads((out/'session_metadata.json').read_text())['trials'],[])
            self.assertIn('MANUAL_START',output.getvalue())

    def test_rejected_request_is_saved(self):
        class RejectedLink(Link):
            def recv(self,*args):
                if not self.parts:raise KeyboardInterrupt()
                return self.parts.pop(0)
        link=RejectedLink(['[CAP] REQUEST request_id=7 t_ms=1000 target_mdeg=-60000',
                           '[CAP] REJECT request_id=7 reason=not_ready hx8=0 imu=1'])
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'session'
            with patch.object(capture.socket,'create_connection',return_value=link),contextlib.redirect_stdout(io.StringIO()):
                capture.main(['--output',str(out)])
            files=list(out.glob('request_*/metadata.json'));self.assertEqual(len(files),1)
            result=json.loads(files[0].read_text())
            self.assertFalse(result['accepted']);self.assertFalse(result['complete'])
            self.assertEqual(result['request']['target_mdeg'],-60000)
            self.assertEqual(result['rejection']['reason'],'not_ready')

    def test_off_on_off_saves_only_enabled_move_and_finishes_pending_off(self):
        class ToggleLink(Link):
            def recv(self,*args):
                if not self.parts:raise KeyboardInterrupt()
                return self.parts.pop(0)
        lines=['[CAP] MODE mode=MANUAL_HX8 record=0',
               '[CAP] MANUAL_START id=1 t_ms=1000 record=OFF',
               '[CAP] MANUAL_END id=1 t_ms=1200 reason=complete',
               '[CAP] MODE mode=CAPTURE record=1']
        lines += [f'[I] {i+1},{1400+i*15},83,0,0,0,0' for i in range(12)]
        lines += ['[CAP] READY t_ms=1590 mode=CAPTURE record=1 ready=1',
                  '[CAP] START id=2 t_ms=1600 target_mdeg=-20000 imu_seq=12 hx_seq=0',
                  '[I] 13,1610,82,1000,0,0,0',
                  '[CAP] RECORD_SWITCH record=0 after_current_move=1',
                  '[CAP] END id=2 t_ms=3000 reason=complete drops=0 imu_errors=0 imu_seq=13 hx_seq=0',
                  '[CAP] MODE mode=MANUAL_HX8 record=0',
                  '[CAP] MANUAL_START id=3 t_ms=3200 record=OFF',
                  '[CAP] MANUAL_END id=3 t_ms=3400 reason=complete']
        link=ToggleLink(lines)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'session'
            with patch.object(capture.socket,'create_connection',return_value=link),contextlib.redirect_stdout(io.StringIO()):
                code=capture.main(['--output',str(out)])
            files=list(out.glob('move_*/metadata.json'))
            self.assertEqual(code,0);self.assertEqual(len(files),1)
            meta=json.loads(files[0].read_text())
            self.assertEqual(meta['start']['id'],2);self.assertTrue(meta['complete'])
            self.assertFalse(link.sent)


if __name__=='__main__':unittest.main()
