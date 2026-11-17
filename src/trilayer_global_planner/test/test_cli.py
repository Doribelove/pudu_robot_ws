#!/usr/bin/env python3
import contextlib,io,json,os,signal,sys,tempfile,time,unittest
from pathlib import Path
import psutil
from trilayer_global_planner.cli import main
from trilayer_global_planner.demo import make_demo
from trilayer_global_planner.process import run_bounded
from trilayer_global_planner.environment import check_environment
from unittest.mock import patch


class CliContract(unittest.TestCase):
    def test_invalid_input_before_any_worker(self):
        with tempfile.TemporaryDirectory() as d:
            base=['--demo','straight','--domain-id','179','--output',str(Path(d)/'out')]
            for extra in [ ['--start','nan','1','0'],['--timeout','0'],['--domain-id','0'] ]:
                with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as caught:main(base+extra)
                self.assertEqual(caught.exception.code,2)
                self.assertFalse((Path(d)/'out').exists())
    def test_unsafe_transport_override_rejected_without_loading(self):
        for name in ('ROS_DISCOVERY_SERVER','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','LD_PRELOAD'):
            with self.assertRaisesRegex(ValueError,name):check_environment({name:'/not/read/or/contact'})
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{'ROS_DISCOVERY_SERVER':'not-contacted'}):
            out=Path(d)/'out'
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as caught:
                main(['--demo','straight','--domain-id','181','--output',str(out)])
            self.assertEqual(caught.exception.code,2);self.assertFalse(out.exists())
    def test_invalid_output_parent(self):
        with tempfile.TemporaryDirectory() as d:
            parent=Path(d)/'file';parent.write_text('retained')
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as caught:
                main(['--demo','straight','--domain-id','181','--output',str(parent/'out')])
            self.assertEqual(caught.exception.code,2);self.assertEqual(parent.read_text(),'retained')
    def test_demo_contains_real_obstacle(self):
        with tempfile.TemporaryDirectory() as d:
            p=make_demo(Path(d)/'obstacle','obstacle')
            raw=(p.parent/'map.pgm').read_bytes().split(b'\n',3)[3]
            self.assertEqual(len(raw),240*240)
            self.assertEqual(raw[150*240+70],0)
            self.assertEqual(raw[150*240+90],254)
    def test_successful_subprocess_status(self):
        with tempfile.TemporaryDirectory() as d:
            r=run_bounded([sys.executable,'-c','print("ok")'],Path(d)/'log',timeout=2.,env=os.environ.copy(),grace=.3)
            self.assertEqual(r['returncode'],0);self.assertFalse(r['timed_out']);self.assertFalse(r['remaining_processes'])
    def test_explicit_child_cwd_excludes_caller_default_xml(self):
        with tempfile.TemporaryDirectory() as d:
            outer=Path(d);clean=outer/'output';clean.mkdir()
            (outer/'DEFAULT_FASTRTPS_PROFILES.xml').write_text('must not be loaded')
            original=os.getcwd()
            try:
                os.chdir(outer)
                code='import os,pathlib; print(os.getcwd()); assert not pathlib.Path("DEFAULT_FASTRTPS_PROFILES.xml").exists()'
                r=run_bounded([sys.executable,'-c',code],outer/'log',timeout=2.,env=os.environ.copy(),cwd=clean)
                self.assertEqual(r['returncode'],0)
                self.assertEqual((outer/'log').read_text().strip(),str(clean))
                self.assertEqual(os.getcwd(),str(outer))
            finally:os.chdir(original)
    def test_cli_supplies_new_output_cwd(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'out'
            execution={'returncode':1,'timed_out':False,'interrupted':False,'remaining_processes':[]}
            with patch('trilayer_global_planner.cli.run_bounded',return_value=execution) as runner,contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--demo','straight','--domain-id','181','--output',str(out)]),1)
            self.assertEqual(runner.call_args.kwargs['cwd'],out.resolve())
            self.assertTrue(out.is_dir())
    def test_timeout_cleans_nested_new_session(self):
        with tempfile.TemporaryDirectory() as d:
            pidfile=Path(d)/'child.pid'
            code='import subprocess,sys,time,signal; signal.signal(signal.SIGINT,signal.SIG_IGN); p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"],start_new_session=True); open(sys.argv[1],"w").write(str(p.pid)); time.sleep(30)'
            t=time.monotonic()
            r=run_bounded([sys.executable,'-c',code,str(pidfile)],Path(d)/'log',timeout=.4,env=os.environ.copy(),grace=.3)
            self.assertTrue(r['timed_out']);self.assertLess(time.monotonic()-t,4)
            self.assertFalse(r['remaining_processes'])
            pid=int(pidfile.read_text())
            if psutil.pid_exists(pid):self.assertEqual(psutil.Process(pid).status(),psutil.STATUS_ZOMBIE)


if __name__=='__main__':unittest.main(verbosity=2)
