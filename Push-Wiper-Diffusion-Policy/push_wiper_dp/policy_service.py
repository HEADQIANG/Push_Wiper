"""Line-oriented inference service for the Push-Wiper policy.

The service keeps the 1.3 GB checkpoint loaded in the policy environment while
the AIRBOT process remains responsible for cameras, robot motion, and force
control.  Requests contain a raw 480x640 uint8 mask encoded as base64.
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np

from .predict import PushWiperPredictor


MASK_SHAPE = (480, 640)
ACTION_DEFINITION_VERSION = 2


def _decode_mask(value) -> np.ndarray:
    if not isinstance(value, str):
        raise ValueError("mask_b64 must be a base64 string")
    try:
        raw = base64.b64decode(value.encode("ascii"), validate=True)
    except Exception as exc:
        raise ValueError("mask_b64 is not valid base64") from exc
    mask = np.frombuffer(raw, dtype=np.uint8)
    if mask.size != MASK_SHAPE[0] * MASK_SHAPE[1]:
        raise ValueError(f"mask must contain {MASK_SHAPE[0] * MASK_SHAPE[1]} uint8 values")
    mask = mask.reshape(MASK_SHAPE)
    if not np.isin(mask, (0, 1)).all():
        raise ValueError("mask must contain only 0 (dirt) and 1 (clean)")
    if not np.any(mask == 0):
        raise ValueError("mask must contain at least one dirt pixel")
    return mask.copy()


def _request(predictor: PushWiperPredictor, payload: dict) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("request must be a JSON object")
    pose = np.asarray(payload.get("capture_reference_pose"), dtype=np.float32)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError("capture_reference_pose must contain seven finite values")
    seed = int(payload.get("seed", 42))
    actions = predictor.predict(_decode_mask(payload.get("mask_b64")), pose, seed=seed)
    if actions.shape != (16, 3) or not np.isfinite(actions).all():
        raise ValueError("policy returned an invalid action array")
    return {
        "actions": actions.tolist(),
        "action_definition_version": ACTION_DEFINITION_VERSION,
        "weights_source": predictor.weights_source,
        "image_size": list(predictor.image_size),
    }


def serve(predictor: PushWiperPredictor) -> int:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        request_id = None
        try:
            payload = json.loads(line)
            request_id = payload.get("request_id") if isinstance(payload, dict) else None
            result = _request(predictor, payload)
            response = {"ok": True, "request_id": request_id, **result}
        except Exception as exc:
            response = {"ok": False, "request_id": request_id, "error": str(exc)}
        sys.stdout.write(json.dumps(response, ensure_ascii=False, separators=(",", ":")) + "\n")
        sys.stdout.flush()
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--validate", action="store_true", help="Load the checkpoint and exit")
    args = parser.parse_args(argv)
    predictor = PushWiperPredictor(args.checkpoint, device=args.device)
    print(
        json.dumps(
            {
                "ready": True,
                "checkpoint": str(predictor.checkpoint),
                "device": str(predictor.device),
                "weights_source": predictor.weights_source,
                "image_size": list(predictor.image_size),
            },
            ensure_ascii=False,
        ),
        file=sys.stderr,
        flush=True,
    )
    if args.validate:
        return 0
    return serve(predictor)


if __name__ == "__main__":
    raise SystemExit(main())
