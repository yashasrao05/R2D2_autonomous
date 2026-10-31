from __future__ import annotations
import json, os, tempfile
from pathlib import Path
from typing import Any


class CheckpointManager:
    def __init__(self, path: str | Path = "storage/mission_checkpoint.json"):
        self.path = Path(path)

    def save(self, state: str, last_cleared_waypoint: int | None, **extra: Any) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"state": state, "last_cleared_waypoint": last_cleared_waypoint, **extra}
        fd, temp = tempfile.mkstemp(prefix="checkpoint-", suffix=".json", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(data, handle, indent=2, sort_keys=True)
                handle.flush(); os.fsync(handle.fileno())
            os.replace(temp, self.path)
        except BaseException:
            if os.path.exists(temp): os.unlink(temp)
            raise

    def load(self) -> dict[str, Any]:
        if not self.path.exists():
            raise FileNotFoundError(f"no checkpoint at {self.path}")
        return json.loads(self.path.read_text(encoding="utf-8"))

    def clear(self) -> None:
        if self.path.exists(): self.path.unlink()
