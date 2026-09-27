"""Manual editing, persistence and strict export regression tests (no hardware)."""

from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from airbot_ie.push_wiper.annotations import (
    annotation_directory, digest, load_annotation,
)
from airbot_ie.push_wiper.collect import run_demo
from airbot_ie.push_wiper.config import CollectionConfig, atomic_json
from airbot_ie.push_wiper.export import MaskConfig, export_dataset
from airbot_ie.push_wiper.review_masks import ReviewImage, Reviewer, Viewport, review_paths


class ReviewTests(unittest.TestCase):
    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.source = self.root / "raw"
        self.annotations = self.root / "annotations"
        self.segment = self.source / "task_1" / "segment_1"
        self.segment.mkdir(parents=True)
        self.path = self.segment / "before.png"
        image = np.full((80, 120, 3), 220, np.uint8)
        image[10:25, 10:25] = [20, 20, 180]
        for name in ("before", "after"):
            self.assertTrue(cv2.imwrite(str(self.segment / f"{name}.png"), image))
        atomic_json(self.segment / "meta.json", {"status": "accepted", "camera": {"roi": None}})

    def frame(self, path=None):
        return ReviewImage(path or self.path, self.annotations, MaskConfig())

    def record_path(self):
        return annotation_directory(self.annotations, self.path) / "review.json"

    def test_paint_erase_undo_and_restore_without_automatic_segmentation(self):
        frame = self.frame()
        self.assertEqual(frame.status, "unreviewed")
        initial = frame.mask.copy()
        frame.paint((50, 50), (70, 50), 2)
        frame.end_stroke()
        self.assertTrue((frame.mask[50, 50:71] == 0).all())
        painted = frame.mask.copy()
        frame.paint((55, 50), (65, 50), 2, erase=True)
        frame.end_stroke()
        self.assertTrue((frame.mask[50, 55:66] == 1).all())
        frame.undo()
        np.testing.assert_array_equal(frame.mask, painted)
        frame.save()
        with patch("airbot_ie.push_wiper.review_masks.stain_mask", side_effect=AssertionError("Recomputed")):
            restored = self.frame()
        np.testing.assert_array_equal(restored.mask, painted)
        self.assertEqual(restored.status, "draft")
        frame.undo()
        np.testing.assert_array_equal(frame.mask, initial)

    def test_confirmed_is_revoked_on_disk_before_edit_and_undo(self):
        frame = self.frame()
        frame.paint((50, 50), (50, 50), 1)
        frame.end_stroke()
        frame.save("confirmed")
        frame.undo()
        self.assertEqual(json.loads(self.record_path().read_text())["status"], "draft")
        frame.save("confirmed")
        frame.begin_stroke()
        self.assertEqual(json.loads(self.record_path().read_text())["status"], "draft")

    def test_failed_revocation_does_not_modify_confirmed_mask(self):
        frame = self.frame()
        frame.save("confirmed")
        initial = frame.mask.copy()
        with patch("airbot_ie.push_wiper.annotations.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                frame.paint((50, 50), (60, 50), 2)
        np.testing.assert_array_equal(frame.mask, initial)
        self.assertEqual(self.frame().status, "confirmed")

    def test_failed_commit_preserves_previous_complete_annotation(self):
        frame = self.frame()
        frame.save("draft")
        initial = frame.mask.copy()
        frame.paint((50, 50), (60, 50), 2)
        frame.end_stroke()
        with patch("airbot_ie.push_wiper.annotations.atomic_json", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                frame.save("confirmed")
        restored = self.frame()
        np.testing.assert_array_equal(restored.mask, initial)
        self.assertEqual(restored.status, "draft")

    def test_corruption_blocks_editing_without_replacing_annotation(self):
        self.frame().save("confirmed")
        record = json.loads(self.record_path().read_text())
        png = self.record_path().with_name(f"mask-{record['mask_sha256']}.png")
        png.write_bytes(b"broken")
        frame = self.frame()
        self.assertEqual(frame.status, "invalid")
        with self.assertRaisesRegex(ValueError, "checksum"):
            frame.begin_stroke()
        self.assertEqual(png.read_bytes(), b"broken")

    def test_source_change_and_roi_change_invalidate_annotation(self):
        for change in ("source", "roi"):
            with self.subTest(change=change):
                frame = self.frame()
                if frame.error:
                    shutil.rmtree(self.annotations)
                    frame = self.frame()
                frame.save("confirmed")
                if change == "source":
                    image = frame.image.copy()
                    image[0, 0] = [0, 0, 0]
                    cv2.imwrite(str(self.path), image)
                else:
                    atomic_json(self.segment / "meta.json", {"status": "accepted", "camera": {"roi": [2, 2, 100, 60]}})
                self.assertEqual(self.frame().status, "invalid")
                with self.assertRaisesRegex(ValueError, "changed"):
                    frame.save()

    def test_roi_native_resolution_is_preserved(self):
        atomic_json(self.segment / "meta.json", {"status": "accepted", "camera": {"roi": [10, 8, 60, 40]}})
        frame = self.frame()
        self.assertEqual(frame.mask.shape, (40, 60))
        frame.save("confirmed")
        np.testing.assert_array_equal(self.frame().mask, frame.mask)
        self.assertEqual(json.loads(self.record_path().read_text())["roi"], [10, 8, 60, 40])

    def test_viewport_mapping_and_letterboxing(self):
        view = Viewport((480, 640))
        self.assertIsNone(view.point(100, 200))  # Original is read-only.
        self.assertEqual(view.point(420, 104), (0, 0))
        self.assertEqual(view.point(840 + 210, 104 + 157), (320, 239))
        view.change_zoom(2)
        self.assertEqual(view.point(420, 104), (160, 120))
        self.assertEqual(view.point(839, 418), (479, 359))
        view.pan(1, 1)
        self.assertEqual(view.point(420, 104), (240, 180))
        square = Viewport((100, 100))
        self.assertIsNone(square.point(420, 104))  # Left letterbox.
        self.assertEqual(square.point(420 + 52, 104), (0, 0))

    def test_autosave_navigation_exit_and_save_failure_stays_on_image(self):
        reviewer = Reviewer(self.source, self.annotations, MaskConfig())
        reviewer.current.paint((50, 50), (60, 50), 2)
        reviewer.current.end_stroke()
        with patch.object(reviewer.current, "save", side_effect=OSError("disk full")):
            self.assertTrue(reviewer.handle_key(ord("n")))
            self.assertEqual(reviewer.index, 0)
            self.assertTrue(reviewer.handle_key(ord("q")))
            self.assertTrue(reviewer.current.dirty)
        reviewer.handle_key(ord("n"))
        self.assertEqual(reviewer.index, 1)
        self.assertEqual(self.frame().status, "draft")
        reviewer.current.paint((50, 50), (60, 50), 2)
        self.assertFalse(reviewer.handle_key(ord("q")))
        self.assertEqual(self.frame(self.path.with_name("after.png")).status, "draft")

    def test_confirm_needs_review_resume_and_toggle_overlay(self):
        reviewer = Reviewer(self.source, self.annotations, MaskConfig())
        reviewer.handle_key(13)
        reviewer.handle_key(ord("n"))
        reviewer.handle_key(ord("u"))
        self.assertEqual(reviewer.current.status, "needs_review")
        reopened = Reviewer(self.source, self.annotations, MaskConfig())
        self.assertEqual(reopened.index, 1)
        self.assertEqual(reopened.current.status, "needs_review")
        initial = reopened.current.mask.copy()
        reopened.handle_key(ord("o"))
        self.assertFalse(reopened.overlay)
        np.testing.assert_array_equal(initial, reopened.current.mask)

    def test_mouse_stroke_and_zoom_paint_on_original_pixel_coordinates(self):
        reviewer = Reviewer(self.source, self.annotations, MaskConfig())
        reviewer.view = Viewport((80, 120))
        reviewer.view.change_zoom(2)
        reviewer.brush = 1
        point = reviewer.view.point(630, 261)
        reviewer.mouse(cv2.EVENT_LBUTTONDOWN, 630, 261, cv2.EVENT_FLAG_LBUTTON, None)
        reviewer.mouse(cv2.EVENT_LBUTTONUP, 630, 261, 0, None)
        x, y = point
        self.assertEqual(reviewer.current.mask[y, x], 0)
        self.assertEqual(len(reviewer.current.undo_stack), 1)

    def test_accepted_only_and_annotations_outside_raw(self):
        for name, status in (("segment_2", "rejected"), ("segment_3.partial", "accepted")):
            path = self.segment.with_name(name)
            path.mkdir()
            atomic_json(path / "meta.json", {"status": status})
        self.assertEqual(review_paths(self.source), [self.path, self.path.with_name("after.png")])
        with self.assertRaisesRegex(ValueError, "outside"):
            Reviewer(self.source, self.source / "annotations", MaskConfig())

    def test_crossing_between_panels_does_not_draw_a_line_across_image(self):
        reviewer = Reviewer(self.source, self.annotations, MaskConfig())
        reviewer.brush = 1
        reviewer.mouse(cv2.EVENT_LBUTTONDOWN, 830, 261, cv2.EVENT_FLAG_LBUTTON, None)
        reviewer.mouse(cv2.EVENT_MOUSEMOVE, 850, 261, cv2.EVENT_FLAG_LBUTTON, None)
        reviewer.mouse(cv2.EVENT_LBUTTONUP, 850, 261, 0, None)
        self.assertEqual(reviewer.current.mask[40, 60], 1)

    def test_window_close_failure_reopens_and_keeps_edits(self):
        reviewer = Reviewer(self.source, self.annotations, MaskConfig())
        reviewer.current.paint((50, 50), (60, 50), 1)
        with (
            patch.object(reviewer, "open_window") as opened,
            patch.object(reviewer, "autosave", side_effect=[OSError("disk full"), None]),
            patch("cv2.getWindowProperty", return_value=0),
            patch("cv2.imshow"),
            patch("cv2.waitKey", return_value=ord("q")),
            patch("cv2.destroyAllWindows"),
        ):
            reviewer.run()
        self.assertEqual(opened.call_count, 2)
        self.assertTrue(reviewer.current.dirty)


class ManualExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = TemporaryDirectory()
        cls.addClassCleanup(cls.fixture.cleanup)
        cls.fixture_raw = Path(cls.fixture.name) / "raw"
        with redirect_stdout(StringIO()):
            run_demo(CollectionConfig(output=str(cls.fixture_raw)))

    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.source = self.root / "raw"
        shutil.copytree(self.fixture_raw, self.source)
        self.annotations = self.root / "annotations"
        self.paths = review_paths(self.source)
        self.sequence = 0

    def confirm(self, path):
        frame = ReviewImage(path, self.annotations, MaskConfig())
        frame.mask[:] = 1
        if path.name == "before.png":
            frame.mask[2, 3] = 0
            frame.mask[7, 9] = 0
        frame.save("confirmed")
        return frame

    def export(self, **kwargs):
        self.sequence += 1
        output = self.root / f"export_{self.sequence}"
        result = export_dataset(self.source, output, allow_simulated=True,
                                annotations=self.annotations, **kwargs)
        return result, output

    def test_both_confirmed_required_even_with_include_review(self):
        self.confirm(self.paths[0])
        result, _ = self.export(include_review=True)
        self.assertEqual(result["exported"], 0)
        self.assertIn("Missing annotation", result["segments"][0]["reason"])
        after = self.confirm(self.paths[1])
        after.save("needs_review")
        result, _ = self.export(include_review=True)
        self.assertEqual(result["exported"], 0)
        self.assertIn("not confirmed", result["segments"][0]["reason"])

    def test_manual_pixels_preserved_statistics_recomputed_raw_unchanged(self):
        hashes = {p: digest(p.read_bytes()) for p in self.source.rglob("*") if p.is_file()}
        before, after = self.confirm(self.paths[0]), self.confirm(self.paths[1])
        with patch("airbot_ie.push_wiper.export.stain_mask", side_effect=AssertionError("Recomputed")):
            result, output = self.export()
        self.assertEqual(result["exported"], 1)
        report = next(s for s in result["segments"] if s["exported"])
        with np.load(output / report["sample"]) as sample:
            np.testing.assert_array_equal(sample["mask"], before.mask)
            np.testing.assert_array_equal(sample["after_mask"], after.mask)
        self.assertEqual(report["quality"]["components"], 2)
        self.assertAlmostEqual(report["quality"]["dirty_fraction"], 2 / before.mask.size)
        meta = json.loads((output / report["sample"]).with_name("meta.json").read_text())
        self.assertEqual(meta["mask_source"], "manual_confirmed")
        for path, checksum in hashes.items():
            self.assertEqual(digest(path.read_bytes()), checksum)

    def test_invalid_annotations_are_excluded_with_reasons(self):
        for fault in ("checksum", "dimensions", "source", "roi", "nonbinary", "missing_png"):
            with self.subTest(fault=fault):
                if self.annotations.exists():
                    shutil.rmtree(self.annotations)
                for path in self.paths[:2]:
                    self.confirm(path)
                record_path = annotation_directory(self.annotations, self.paths[0]) / "review.json"
                record = json.loads(record_path.read_text())
                png = record_path.with_name(f"mask-{record['mask_sha256']}.png")
                if fault == "checksum":
                    png.write_bytes(b"corrupt")
                elif fault == "dimensions":
                    record["shape"] = [1, 1]
                elif fault == "source":
                    record["source_sha256"] = "0" * 64
                elif fault == "roi":
                    record["roi"] = [0, 0, 5, 5]
                elif fault == "nonbinary":
                    pixels = np.full(record["shape"], 127, np.uint8)
                    payload = cv2.imencode(".png", pixels)[1].tobytes()
                    record["mask_sha256"] = digest(payload)
                    record_path.with_name(f"mask-{record['mask_sha256']}.png").write_bytes(payload)
                else:
                    png.unlink()
                atomic_json(record_path, record)
                result, _ = self.export(include_review=True)
                self.assertEqual(result["exported"], 0)
                self.assertTrue(result["segments"][0]["reason"])

    def test_task_split_and_existing_quality_checks_remain(self):
        for path in self.paths:
            self.confirm(path)
        result, _ = self.export()
        self.assertEqual(result["exported"], 4)
        groups = {}
        for row in result["segments"]:
            groups.setdefault(row["task_id"], set()).add(row["split"])
        self.assertTrue(all(len(splits) == 1 for splits in groups.values()))
        result, _ = self.export(max_gap_s=0.001)
        self.assertEqual(result["exported"], 0)
        self.assertIn("sampling_gap", result["segments"][0]["reason"])

    def test_automatic_export_does_not_read_manual_annotations(self):
        self.confirm(self.paths[0])
        with patch("airbot_ie.push_wiper.export.load_annotation", side_effect=AssertionError("Manual read")):
            result = export_dataset(self.source, self.root / "automatic", allow_simulated=True)
        self.assertEqual(result["exported"], 4)


if __name__ == "__main__":
    unittest.main()
