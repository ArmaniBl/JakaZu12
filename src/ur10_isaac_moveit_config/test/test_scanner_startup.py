"""Projector selection never silently chooses a primary or ambiguous display."""
import importlib.util
from pathlib import Path
import unittest
import tempfile

HELPER=Path(__file__).resolve().parents[3]/'integrations/scanner_atomic.py'
spec=importlib.util.spec_from_file_location('scanner_atomic',HELPER)
atomic=importlib.util.module_from_spec(spec);spec.loader.exec_module(atomic)


class ScannerStartupTests(unittest.TestCase):
    def test_exposure_override_and_invalid_settings(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json'
            self.assertEqual(atomic.capture_exposure(100000,path),100000)
            path.write_text('{"exposure_us": 40000}')
            self.assertEqual(atomic.capture_exposure(100000,path),40000)
            for value in ('0','100001','true','"NaN"'):
                path.write_text('{"exposure_us":'+value+'}')
                with self.assertRaises(ValueError):atomic.capture_exposure(100000,path)
    def setUp(self):
        self.primary=dict(id=1,builtin=True,main=True,mirrors=0)
        self.projector=dict(id=3,builtin=False,main=False,mirrors=0)
    def test_auto_uses_only_external_projector(self):
        self.assertEqual(atomic.select_projector([self.primary,self.projector],0),self.projector)
    def test_configured_id_is_checked(self):
        self.assertEqual(atomic.select_projector([self.primary,self.projector],3),self.projector)
        with self.assertRaises(ValueError):atomic.select_projector([self.primary,self.projector],1)
    def test_missing_and_ambiguous_projectors_are_rejected(self):
        for displays in ([self.primary],[self.projector,dict(self.projector,id=4)]):
            with self.assertRaises(ValueError):atomic.select_projector(displays,0)
    def test_transient_file_lock_is_retried_but_permanent_error_is_not_hidden(self):
        class Writer:
            def __init__(self):self.calls=0
            def replace(self,target):
                self.calls+=1
                if self.calls<3:raise PermissionError('reader holds file')
                return target
        writer=Writer()
        self.assertEqual(atomic.replace_when_available(writer,'command'),'command')
        self.assertEqual(writer.calls,3)
        with self.assertRaises(PermissionError):atomic.replace_when_available(Writer(),'command',timeout=0)

    def test_ready_read_retries_a_transient_lock(self):
        class Reader:
            calls=0
            def read_text(self):
                self.calls+=1
                if self.calls<3:raise PermissionError('projector is replacing ready')
                return '23'
        reader=Reader()
        self.assertEqual(atomic.read_text_when_available(reader),'23')
        self.assertEqual(reader.calls,3)
        with self.assertRaises(PermissionError):atomic.read_text_when_available(Reader(),timeout=0)


if __name__=='__main__':unittest.main()
