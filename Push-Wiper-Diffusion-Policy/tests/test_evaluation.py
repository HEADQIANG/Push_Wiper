"""Metric semantics and deterministic inference independent of a trained model."""

import json
import random

import numpy as np
import pytest
import torch

from push_wiper_dp.evaluation import evaluate_policy, predict_observation, trajectory_metrics


class ObservationOnlyPolicy(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.anchor = torch.nn.Parameter(torch.zeros(()))

    def predict_action(self, obs):
        assert set(obs) == {"mask", "capture_reference_pose"}
        assert obs["mask"].shape == (1, 1, 3, 4, 6)
        return {"action": torch.randn(1, 16, 3, device=self.anchor.device)}


def observation():
    return {"mask": torch.ones(1, 3, 4, 6), "capture_reference_pose": torch.zeros(1, 7)}


def test_metric_units_and_circular_yaw_error():
    target = np.zeros((16, 3))
    prediction = np.tile([0.003, 0.004, 2 * np.pi + np.deg2rad(10)], (16, 1))
    metrics = trajectory_metrics(prediction, target)
    assert metrics["xy_ade_mm"] == pytest.approx(5)
    assert metrics["xy_fde_mm"] == pytest.approx(5)
    assert metrics["yaw_mae_deg"] == pytest.approx(10)
    assert metrics["xy_max_step_mm"] == 0
    assert metrics["yaw_max_step_deg"] == 0


def test_continuity_uses_unwrapped_yaw():
    prediction = np.zeros((16, 3))
    prediction[1:, 0] = 0.02
    prediction[1:, 2] = 2 * np.pi
    metrics = trajectory_metrics(prediction, np.zeros_like(prediction))
    assert metrics["xy_max_step_mm"] == pytest.approx(20)
    assert metrics["yaw_max_step_deg"] == pytest.approx(360)
    assert metrics["yaw_mae_deg"] == pytest.approx(0, abs=1e-10)


@pytest.mark.parametrize("bad", [np.zeros((16, 2)), np.zeros((1, 3)), np.full((16, 3), np.nan)])
def test_reject_invalid_trajectories(bad):
    with pytest.raises(ValueError):
        trajectory_metrics(bad, np.zeros((16, 3)))


def test_prediction_preserves_rng_and_training_mode():
    policy = ObservationOnlyPolicy().train()
    torch_state = torch.random.get_rng_state().clone()
    numpy_state = np.random.get_state()
    python_state = random.getstate()
    first = predict_observation(policy, observation(), 42)
    second = predict_observation(policy, observation(), 42)
    assert np.array_equal(first, second)
    assert first.shape == (16, 3)
    assert first.dtype == np.float32
    assert policy.training
    assert torch.equal(torch_state, torch.random.get_rng_state())
    actual_numpy_state = np.random.get_state()
    assert numpy_state[0] == actual_numpy_state[0]
    assert np.array_equal(numpy_state[1], actual_numpy_state[1])
    assert numpy_state[2:] == actual_numpy_state[2:]
    assert python_state == random.getstate()


def test_evaluation_writes_predictions_metrics_and_separate_panels(tmp_path):
    policy = ObservationOnlyPolicy()
    dataset = [{"obs": observation(), "action": torch.zeros(16, 3)} for _ in range(2)]
    report = evaluate_policy(policy, dataset, tmp_path)
    assert report["count"] == 2
    assert [sample["seed"] for sample in report["samples"]] == [1042, 1043]
    with np.load(tmp_path / "predictions.npz", allow_pickle=False) as saved:
        assert saved["predictions"].shape == (2, 16, 3)
        assert saved["predictions"].dtype == np.float32
        assert saved["targets"].shape == (2, 16, 3)
    assert json.loads((tmp_path / "metrics.json").read_text())["count"] == 2
    assert len(list((tmp_path / "plots").glob("*.png"))) == 2
    for key in ("xy_max_step_mm", "yaw_max_step_deg"):
        assert report["aggregate"][key] == max(sample[key] for sample in report["samples"])
