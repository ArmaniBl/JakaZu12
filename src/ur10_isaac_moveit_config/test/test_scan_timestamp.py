"""Last cloud update is informational and survives panel restarts."""
import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from welding_sim.scan_timestamp import latest_update


class ScanTimestampTests(unittest.TestCase):
    def test_missing_timestamp_is_unknown(self):
        with tempfile.TemporaryDirectory() as root:self.assertIsNone(latest_update(root))

    def test_old_timestamp_is_reported_without_freshness_gate(self):
        with tempfile.TemporaryDirectory() as root:
            scans=Path(root)/'scans';scans.mkdir()
            (scans/'scan_status.json').write_text(json.dumps(dict(updated_unix=100.)))
            self.assertEqual(latest_update(root),100.)

    def test_existing_cloud_cache_is_used_before_first_new_scan(self):
        with tempfile.TemporaryDirectory() as root:
            scans=Path(root)/'scans';scans.mkdir();stamp=time.time()-600
            np.savez(scans/'preview-cloud-cache.npz',updated_unix=stamp)
            self.assertEqual(latest_update(root),stamp)

    def test_invalid_and_future_values_are_not_displayed(self):
        with tempfile.TemporaryDirectory() as root:
            scans=Path(root)/'scans';scans.mkdir()
            for value in (True,None,'bad',float('nan'),time.time()+3600):
                (scans/'scan_status.json').write_text(json.dumps(dict(updated_unix=value)))
                self.assertIsNone(latest_update(root))


if __name__=='__main__':unittest.main()
