"""Progress must survive real diagnostic rows larger than the old 8 kB cap."""

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from run import latest_trace_row


class Progress(unittest.TestCase):
    def test_large_complete_native_row_is_visible(self):
        row = {"t": 14.45, "info": {"physics": {"diagnostic": "x" * 30000}}}
        with TemporaryDirectory() as directory:
            path = Path(directory) / "physics.jsonl"
            path.write_text(json.dumps(row) + "\n")
            self.assertEqual(latest_trace_row(path), row)

    def test_partial_concurrent_write_keeps_previous_complete_measurement(self):
        row = {"t": 14.40, "info": {"physics": {"diagnostic": "x" * 20000}}}
        with TemporaryDirectory() as directory:
            path = Path(directory) / "physics.jsonl"
            path.write_text(
                json.dumps(row) + "\n" + json.dumps({"t": 14.45, "info": {"diagnostic": "y" * 20000}})[:-5]
            )
            self.assertEqual(latest_trace_row(path), row)

    def test_empty_trace_has_no_invented_progress(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "physics.jsonl"
            path.write_text("")
            self.assertIsNone(latest_trace_row(path))
