"""Synthetic split sentinels catch leakage, corrupt labels and relocation bugs."""

import json
import shutil

import numpy as np
import pytest
import torch

from push_wiper_dp.dataset import ACTION_DEFINITION, PushWiperDataset, preprocess_observation


@pytest.fixture
def dataset_root(tmp_path):
    root = tmp_path / "dataset"
    root.mkdir()
    samples = []
    for index, (task, split, offset) in enumerate([
        ("train_task", "train", 0.0),
        ("train_task", "train", 1.0),
        ("held_out_task", "validation", 100.0),
    ]):
        relative = f"{split}/{task}/segment_{index}/sample.npz"
        destination = root / relative
        destination.parent.mkdir(parents=True)
        mask = np.ones((480, 640), dtype=np.uint8)
        mask[20:40, 30:60] = 0
        actions = np.stack([
            np.linspace(offset, offset + 0.5, 16),
            np.linspace(offset + 0.5, offset, 16),
            np.linspace(3.0, 4.0, 16),
        ], axis=-1).astype(np.float32)
        np.savez_compressed(
            destination,
            mask=mask,
            after_mask=np.ones_like(mask),
            capture_reference_pose=np.array([0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
            actions=actions,
            action_definition_version=np.int64(2),
        )
        samples.append({"task_id": task, "segment_id": f"segment_{index}", "split": split, "sample": relative})
    manifest = {"action_definition": ACTION_DEFINITION, "samples": samples, "validation_tasks": ["held_out_task"]}
    (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return root


def _change_sample(root, **changes):
    manifest = json.loads((root / "manifest.json").read_text())
    path = root / manifest["samples"][0]["sample"]
    with np.load(path, allow_pickle=False) as sample:
        values = dict(sample)
    values.update(changes)
    np.savez_compressed(path, **values)


def _change_manifest(root, change):
    path = root / "manifest.json"
    manifest = json.loads(path.read_text())
    change(manifest)
    path.write_text(json.dumps(manifest))


def test_complete_segment_interface_and_preprocessing(dataset_root):
    dataset = PushWiperDataset(dataset_root)
    sample = dataset[0]
    assert len(dataset) == 2
    assert len(dataset.get_validation_dataset()) == 1
    assert sample["obs"]["mask"].shape == (1, 3, 240, 320)
    assert sample["obs"]["capture_reference_pose"].shape == (1, 7)
    assert sample["action"].shape == (16, 3)
    assert sample["action"].dtype == torch.float32
    assert set(torch.unique(sample["obs"]["mask"]).tolist()) == {0, 1}
    assert sample["action"][-1, 2] == 4.0  # do not wrap the exported yaw
    assert dataset.get_all_actions().shape == (32, 3)
    assert dataset.describe()["splits"]["train"] == {"samples": 2, "tasks": 1}
    json.dumps(dataset.describe())
    sample["action"].fill_(999)
    assert dataset[0]["action"][0, 0] == 0  # returned tensors must not mutate cache


def test_train_only_normalization_constants_and_round_trip(dataset_root):
    dataset = PushWiperDataset(dataset_root)
    normalizer = dataset.get_normalizer()
    train_actions = dataset.get_all_actions()
    normalized = normalizer["action"].normalize(train_actions)
    torch.testing.assert_close(normalized.amin(dim=0), torch.full((3,), -1.0))
    torch.testing.assert_close(normalized.amax(dim=0), torch.ones(3))
    torch.testing.assert_close(normalizer["action"].unnormalize(normalized), train_actions)
    # Deliberately extreme held-out positions must not expand fitted limits.
    val = dataset.get_validation_dataset()
    assert normalizer["action"].normalize(val[0]["action"])[:, :2].min() > 1
    torch.testing.assert_close(
        val.get_normalizer()["action"].normalize(train_actions), normalized
    )
    pose = normalizer["capture_reference_pose"].normalize(dataset[0]["obs"]["capture_reference_pose"])
    torch.testing.assert_close(pose, torch.zeros((1, 7)))
    assert torch.isfinite(pose).all()
    pixels = normalizer["mask"].normalize(dataset[0]["obs"]["mask"])
    assert set(torch.unique(pixels).tolist()) == {-1, 1}
    assert all(not parameter.requires_grad for parameter in normalizer.parameters())


def test_after_mask_is_ignored_and_fingerprint_survives_relocation(dataset_root, tmp_path):
    before = PushWiperDataset(dataset_root)
    # Even a malformed unused after_mask must not become a model input.
    _change_sample(dataset_root, after_mask=np.array([np.nan]))
    after = PushWiperDataset(dataset_root)
    assert before.fingerprint == after.fingerprint
    torch.testing.assert_close(before[0]["obs"]["mask"], after[0]["obs"]["mask"])
    relocated = tmp_path / "relocated"
    shutil.copytree(dataset_root, relocated)
    assert PushWiperDataset(relocated).fingerprint == before.fingerprint
    _change_sample(relocated, actions=np.ones((16, 3), dtype=np.float32))
    assert PushWiperDataset(relocated).fingerprint != before.fingerprint


def test_task_split_leakage_is_rejected(dataset_root):
    _change_manifest(dataset_root, lambda m: m["samples"][2].update(task_id="train_task"))
    with pytest.raises(ValueError, match="task split leakage"):
        PushWiperDataset(dataset_root)


def test_manifest_order_changes_resume_fingerprint(dataset_root):
    before = PushWiperDataset(dataset_root)
    _change_manifest(dataset_root, lambda m: m["samples"].reverse())
    after = PushWiperDataset(dataset_root)
    assert before.fingerprint != after.fingerprint
    assert before.records[0]["segment_id"] != after.records[0]["segment_id"]
    assert sorted(r["segment_id"] for r in before.records) == sorted(
        r["segment_id"] for r in after.records
    )


@pytest.mark.parametrize("changes, message", [
    ({"action_definition_version": np.int64(1)}, "action_definition_version"),
    ({"actions": np.zeros((15, 3))}, "expected shape"),
    ({"actions": np.full((16, 3), np.nan)}, "finite"),
    ({"capture_reference_pose": np.zeros(8)}, "expected shape"),
    ({"mask": np.ones((480, 640), dtype=np.uint8)}, "no dirt"),
    ({"mask": np.full((480, 640), 255, dtype=np.uint8)}, "binary"),
    ({"mask": np.full((480, 640), 1.0 + 1e-10, dtype=np.float64)}, "binary"),
    ({"mask": np.zeros((240, 320), dtype=np.uint8)}, "expected shape"),
])
def test_invalid_samples_are_not_repaired(dataset_root, changes, message):
    _change_sample(dataset_root, **changes)
    with pytest.raises(ValueError, match=message):
        PushWiperDataset(dataset_root)


@pytest.mark.parametrize("path", ["../escape.npz", "/tmp/escape.npz", "train/../../escape.npz"])
def test_unsafe_paths_are_rejected(dataset_root, path):
    _change_manifest(dataset_root, lambda m: m["samples"][0].update(sample=path))
    with pytest.raises(ValueError, match="safe relative path"):
        PushWiperDataset(dataset_root)


def test_symlink_escape_is_rejected(dataset_root, tmp_path):
    external = tmp_path / "outside.npz"
    external.write_bytes(b"not an npz")
    (dataset_root / "escape.npz").symlink_to(external)
    _change_manifest(dataset_root, lambda m: m["samples"][0].update(sample="escape.npz"))
    with pytest.raises(ValueError, match="escapes dataset root"):
        PushWiperDataset(dataset_root)


def test_wrong_action_semantics_are_rejected(dataset_root):
    _change_manifest(dataset_root, lambda m: m["action_definition"].update(quaternion_order="wxyz"))
    with pytest.raises(ValueError, match="action_definition"):
        PushWiperDataset(dataset_root)


def test_duplicate_samples_are_rejected(dataset_root):
    _change_manifest(dataset_root, lambda m: m["samples"].append(m["samples"][0]))
    with pytest.raises(ValueError, match="duplicate sample"):
        PushWiperDataset(dataset_root)


def test_validation_task_list_disagreement_is_rejected(dataset_root):
    _change_manifest(dataset_root, lambda m: m.update(validation_tasks=["wrong_task"]))
    with pytest.raises(ValueError, match="validation_tasks disagrees"):
        PushWiperDataset(dataset_root)


def test_preprocess_matches_dataset_and_preserves_aspect_ratio(dataset_root):
    dataset = PushWiperDataset(dataset_root)
    with np.load(dataset_root / dataset.records[0]["sample"], allow_pickle=False) as sample:
        observation = preprocess_observation(sample["mask"], sample["capture_reference_pose"])
        for key, value in observation.items():
            torch.testing.assert_close(value, dataset[0]["obs"][key])
        with pytest.raises(ValueError, match="height:width"):
            preprocess_observation(sample["mask"], sample["capture_reference_pose"], (96, 96))
