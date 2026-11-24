"""Scene replacement is validated and cannot overlap planning or execution."""
from pathlib import Path
import subprocess
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from welding_sim.web_control import Controller
from welding_sim.profiles import JAKA


class WebWorkpieceTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.controller=Controller(self.directory.name,Path(self.directory.name)/'weld.py')
        self.controller.telemetry['moveit']=True
        self.controller.last_telemetry=time.monotonic()

    def tearDown(self):
        self.directory.cleanup()

    def test_presets_use_fixed_argv_and_replace_selected_id(self):
        for shape in ('cad:02-tee-gusset','cad:04-pipe-plate','cad:05-wave-butt'):
            with patch('welding_sim.web_control.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run:
                result=self.controller.replace_workpiece({'object':'custom_part','shape':shape})
                argv=run.call_args.args[0]
                self.assertEqual(argv[argv.index('--id')+1],'custom_part')
                self.assertIn('--clear-markers',argv)
                self.assertEqual(result['shape'],shape)
                self.assertNotIn('shell',run.call_args.kwargs)
                self.assertEqual(argv[argv.index('--catalog-model')+1],shape.removeprefix('cad:'))

    def test_real_catalog_models_stand_on_table(self):
        self.controller.profile=JAKA
        for shape in ('cad:02-tee-gusset','cad:04-pipe-plate','cad:05-wave-butt'):
            with patch('welding_sim.web_control.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run:
                self.controller.replace_workpiece({'shape':shape})
                argv=run.call_args.args[0]
                index=argv.index('--center')+1
                self.assertEqual([float(v) for v in argv[index:index+3]],[-.225,-.74,.005])

    def test_unknown_form_and_no_scene_are_rejected(self):
        with patch('welding_sim.web_control.subprocess.run') as run:
            for data in ({'shape':'../../file'}, {'shape':'box'}, {'shape':'cylinder'}, {'shape':'prism'}, {'shape':'cad:04-pipe-plate','command':'x'}, []):
                with self.assertRaises(ValueError):self.controller.replace_workpiece(data)
            self.controller.telemetry['moveit']=False
            with self.assertRaises(ValueError):self.controller.replace_workpiece({'shape':'cad:04-pipe-plate'})
            run.assert_not_called()

    def test_active_motion_and_planning_block_replacement(self):
        self.controller.job_process=Mock();self.controller.job_process.poll.return_value=None
        for mode in ('execute','plan'):
            self.controller.job={'mode':mode}
            with patch('welding_sim.web_control.subprocess.run') as run:
                with self.assertRaisesRegex(ValueError,'Нельзя менять'):self.controller.replace_workpiece({'shape':'cad:04-pipe-plate'})
                run.assert_not_called()

    def test_service_failure_and_timeout_are_reported(self):
        for failure in (SimpleNamespace(returncode=1,stdout='',stderr='service unavailable'),
                        subprocess.TimeoutExpired('loader',80)):
            with patch('welding_sim.web_control.subprocess.run') as run:
                if isinstance(failure,Exception):run.side_effect=failure
                else:run.return_value=failure
                with self.assertRaises(ValueError):self.controller.replace_workpiece({'shape':'cad:04-pipe-plate'})


if __name__=='__main__':unittest.main()
