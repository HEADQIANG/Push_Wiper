"""Collection FSM independent from the controller's motion FSM."""

import time
from copy import deepcopy
from pathlib import Path

from .config import WorkHeightReference, atomic_json
from .controller import CommandError
from .geometry import ACTION_DEFINITION
from .recording import SegmentWriter, identifier


class CollectionSession:
    def __init__(
        self, config, references, simulated=False, writer_factory=SegmentWriter
    ):
        self.config = config
        self.references = references
        self.simulated = simulated
        self.writer_factory = writer_factory
        self.task = None
        self.task_directory = None
        self.writer = None
        self.phase = "idle"
        self.last_result = None
        self.started_monotonic = None
        self.seen_events = set()

    def new_task(self):
        if self.task is not None or self.writer is not None:
            raise CommandError("End the current task with T before creating another")
        if not self.references.get("observation"):
            raise CommandError("Missing reference: observation (O); register before N")
        if self.references.get("work_height") is None:
            raise CommandError(
                "Missing work height (Z); move to the surface and press Z before N"
            )
        WorkHeightReference.model_validate(self.references["work_height"])
        self.task = {
            "task_id": identifier("task"),
            "stain": self.config.stain,
            "operator": self.config.operator,
            "status": "active",
            "simulated": self.simulated,
            "references": deepcopy(self.references),
            "action_definition": deepcopy(ACTION_DEFINITION),
            "config": self.config.model_dump(mode="json"),
        }
        self.task_directory = Path(self.config.output) / self.task["task_id"]
        self.task_directory.mkdir(parents=True, exist_ok=False)
        atomic_json(self.task_directory / "task.json", self.task)

    def allow_reference_update(self):
        if self.task is not None or self.writer is not None:
            raise CommandError(
                "Reference changes are only allowed between tasks; finish the task with T first"
            )

    def _observing(self, snapshot):
        if snapshot.get("active_command"):
            raise CommandError("Wait for the robot command before capturing")
        if snapshot["state"] != "observing":
            raise CommandError(
                "Press R and wait for precise observation hold before capturing"
            )
        if not snapshot.get("samples"):
            raise CommandError("Robot feedback unavailable")

    def capture(self, snapshot, frame, stamp_ns=None):
        self._observing(snapshot)
        completed = [
            event["t_ns"]
            for event in snapshot.get("events", [])
            if event["name"] in {"reset_completed", "observation_recovered"}
        ]
        if completed and frame["t"] < completed[-1]:
            raise CommandError("Wait for a fresh camera frame after observation stabilizes")
        if self.task is None:
            raise CommandError("Create a task with N first")
        if self.phase == "idle":
            self.writer = self.writer_factory(
                self.task_directory,
                {
                    "task_id": self.task["task_id"],
                    "stain": self.task["stain"],
                    "simulated": self.simulated,
                    "references": deepcopy(self.task["references"]),
                    "config": self.config.model_dump(mode="json"),
                    "action_reference": "follower SDK end reference in follower base; no sponge TCP offset",
                    "action_definition": deepcopy(ACTION_DEFINITION),
                    "quaternion_order": "xyzw",
                    "position_unit": "m",
                    "angle_unit": "rad",
                    "camera": {"color_order": "bgr", "roi": self.config.camera.roi},
                },
            )
            self.phase = "approach"
            self.started_monotonic = time.monotonic()
            self.seen_events = {
                (event["name"], event["t_ns"]) for event in snapshot.get("events", [])
            }
            self.writer.image("before", frame, snapshot)
            self.sample(snapshot, frame, stamp_ns)
        elif self.phase == "return":
            self.sample(snapshot, frame, stamp_ns)
            self.writer.image("after", frame, snapshot)
            self.phase = "review"
        else:
            raise CommandError("C is available only before a stroke or after returning")

    def toggle_push(self, snapshot, frame, stamp_ns=None):
        if snapshot["state"] != "following":
            raise CommandError("Enable motion with G before marking a stroke")
        if self.phase not in {"approach", "pushing"}:
            raise CommandError("Capture before image first; one stroke per segment")
        stamp = self.sample(snapshot, frame, stamp_ns)
        if self.phase == "approach":
            self.writer.metadata["push_start_ns"] = stamp
            self.writer.event("push_started", stamp)
            self.phase = "pushing"
        else:
            if stamp <= self.writer.metadata["push_start_ns"]:
                raise CommandError("Stroke must have positive duration")
            self.writer.metadata["push_end_ns"] = stamp
            self.writer.event("push_ended", stamp)
            self.phase = "return"

    def allow_reset(self):
        if self.phase not in {"idle", "return"}:
            raise CommandError(
                "End the stroke with Space before reset; finish/reject pending segment first"
            )

    def allow_follow(self):
        if self.phase not in {"idle", "approach"}:
            raise CommandError("Finish/reject this segment before enabling motion")

    def sample(self, snapshot, frame, stamp_ns=None):
        if self.writer is None:
            return None
        if snapshot.get("active_command"):
            raise CommandError("Wait for the robot command before sampling")
        if snapshot["state"] == "fault":
            self.abort("Controller fault: " + snapshot.get("error", ""))
            raise RuntimeError("Controller fault invalidated segment")
        if time.monotonic() - self.started_monotonic >= self.config.max_segment_s:
            self.abort("Segment duration limit reached")
            raise CommandError("Recording limit reached; segment marked incomplete")
        for event in snapshot.get("events", []):
            key = (event["name"], event["t_ns"])
            if key not in self.seen_events:
                self.writer.event(
                    event["name"],
                    event["t_ns"],
                    **{k: v for k, v in event.items() if k not in {"name", "t_ns"}},
                )
                self.seen_events.add(key)
        return self.writer.append(snapshot, frame, self.phase, stamp_ns)

    def finish_segment(self, accepted=True):
        if self.writer is None:
            raise CommandError("No pending segment")
        if accepted and self.phase != "review":
            raise CommandError("End stroke and capture after image before accepting")
        self.last_result = self.writer.finish(
            "accepted" if accepted else "rejected",
            "" if accepted else "Operator rejected",
        )
        self.writer = None
        self.phase = "idle"
        return self.last_result

    def abort(self, reason):
        if self.writer is not None:
            writer, self.writer = self.writer, None
            try:
                self.last_result = writer.finish("incomplete", reason)
            finally:
                self.phase = "idle"

    def end_task(self, interrupted=False):
        if self.writer is not None:
            raise CommandError("Accept or reject pending segment before ending task")
        if self.task is None:
            raise CommandError("No active task")
        self.task["status"] = "interrupted" if interrupted else "completed"
        atomic_json(self.task_directory / "task.json", self.task)
        self.task = None
        self.phase = "idle"
