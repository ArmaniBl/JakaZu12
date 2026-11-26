import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from welding_sim.scan_progress import progress_payload, table_progress


class ScanProgressTests(unittest.TestCase):
    def test_capture_counts_and_real_attempt_times(self):
        state=dict(job=dict(state='CAPTURING',capture_id='new',progress='Pattern 29/41 captured'),
                   active_task=dict(captures=[dict(id='old',started_at='2026-01-01T00:00:00Z'),
                       dict(id='new',started_at='2026-11-19T12:00:00+00:00')]))
        result=progress_payload(state,now=1795090000)
        self.assertEqual((result['completed_frames'],result['total_frames'],result['percent']),(29,41,71))
        self.assertEqual(result['started_unix'],1795089600)

    def test_reconstruction_has_no_invented_percentage(self):
        result=progress_payload(dict(job=dict(state='DECODING',progress='Pattern 41/41 captured')))
        self.assertIsNone(result['percent'])
        self.assertEqual(result['state'],'DECODING')

    def test_invalid_progress_is_indeterminate(self):
        for text in ['Pattern 42/41 captured','Pattern 1/0 captured','Connecting cameras','<script>']:
            self.assertIsNone(progress_payload(dict(job=dict(state='CAPTURING',progress=text)))['percent'])

    def test_active_table_result_supersedes_legacy_failure_for_the_same_scan(self):
        with TemporaryDirectory() as directory:
            scans = Path(directory) / 'scans'
            scans.mkdir()
            report = scans / 'table-stereo-progress.json'
            failed = progress_payload({'job': {'state': 'FAILED', 'capture_id': 'own'}}, now=1000)
            for phase in ('DECODING', 'COMPLETE', 'FAILED'):
                report.write_text(json.dumps(dict(capture_id='own', state=phase, observed_unix=999)))
                self.assertEqual(table_progress(directory, failed)['state'], phase)
            report.write_text(json.dumps(dict(capture_id='other', state='COMPLETE', observed_unix=999)))
            self.assertEqual(table_progress(directory, failed)['state'], 'FAILED')

    def test_failed_scan_is_not_complete(self):
        result=progress_payload(dict(job=dict(state='FAILED',capture_id='scan')))
        self.assertEqual(result['state'],'FAILED')
        self.assertIsNone(result['percent'])


if __name__=='__main__':unittest.main()
