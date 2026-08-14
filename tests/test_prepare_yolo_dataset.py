import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/prepare_yolo_dataset.py"
SPEC = importlib.util.spec_from_file_location("prepare_yolo_dataset", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class LabelValidationTests(unittest.TestCase):
    def validate(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "label.txt"
            path.write_text(text, encoding="utf-8")
            return MODULE.validate_label(path)

    def test_valid_three_classes(self):
        classes, errors = self.validate("0 0.5 0.5 1 1\n1 0.2 0.3 0.1 0.2\n2 0.8 0.8 0.2 0.2\n")
        self.assertEqual(classes, (0, 1, 2))
        self.assertEqual(errors, [])

    def test_rejects_invalid_class_and_crossing_box(self):
        _, errors = self.validate("3 0.1 0.5 0.4 0.2\n")
        self.assertTrue(any("class_id" in error for error in errors))
        self.assertTrue(any("boundary" in error for error in errors))

    def test_rejects_bad_shape_and_negative_size(self):
        _, errors = self.validate("0 0.5 0.5 -0.1 0.2\n0 0.5 0.5 0.1\n")
        self.assertTrue(any("positive" in error for error in errors))
        self.assertTrue(any("5 fields" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
