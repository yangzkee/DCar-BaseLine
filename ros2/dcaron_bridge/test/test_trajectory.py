"""Trajectory CLI validation must work without ROS and reject misleading formats."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

from dcaron_bridge.trajectory import load_trajectory, main


class TrajectoryFileTests(unittest.TestCase):
    def load_text(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.yaml'
            path.write_text(text, encoding='utf-8')
            return load_trajectory(path)

    def test_authored_examples_and_dry_run(self):
        examples = Path(__file__).resolve().parents[1] / 'examples'
        for name, count in [('square', 8), ('stadium', 4)]:
            path = examples / f'{name}.yaml'
            specs, timeout = load_trajectory(path)
            self.assertEqual((len(specs), timeout), (count, 60))
            self.assertTrue(all(spec.profile == 18 for spec in specs))
            with contextlib.redirect_stdout(io.StringIO()) as output:
                main([str(path)])
            self.assertIn('no serial or ROS connection', output.getvalue())

    def test_default_grade_and_zero_server_timeout(self):
        specs, timeout = self.load_text('timeout: 0\nsegments:\n  - {command: 100, x: 0.1, speed: 0.06}\n')
        self.assertEqual((specs[0].profile, timeout), (18, 0))

    def test_wrong_format_empty_and_invalid_values(self):
        invalid = [
            '- {command: 100, x: 0.1, speed: 0.06}',
            'segments: []',
            'segments: [not-a-segment]',
            'timeout: .nan\nsegments: [{command: 100, speed: 0.1}]',
            'timeout: -1\nsegments: [{command: 100, speed: 0.1}]',
            'segments: [{command: 100, speed: 0.1, profile: 2}]',
            'segments: [{command: 100, speed: -.1}]',
        ]
        for text in invalid:
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.load_text(text)


if __name__ == '__main__':
    unittest.main()
