"""Tkinter-based button panel.

This module provides a small API inspired by ``pynput.keyboard.Listener`` style:
- Create a ``Listener`` with callbacks.
- Provide a button layout (rows/columns) to auto-generate a button grid.
- Call ``start()`` to create the window.
- Call ``update()`` periodically to pump UI events.
- Mouse clicks on buttons trigger callbacks with a button name.
- Closing the window triggers an optional close callback.

Notes:
- Tkinter must run on the thread that created the Tk root window. For stability,
    this implementation does NOT run Tk in a background thread.
"""

from math import ceil
from pathlib import Path
from queue import Queue
import re
import shutil
import signal
import shlex
import subprocess
import sys
import threading
from typing import Callable, Dict, List, Optional, Sequence, Union
from typing_extensions import Self
from pydantic import BaseModel, ConfigDict, Field, model_validator
import time
import tkinter as tk


ButtonName = str
ButtonCallback = Union[Callable[[], None], str]
"""Button callback type.

Supported forms:

- Callable: A zero-arg function, called when the button is pressed.
- str: A special string action.

        - Plain text: The text will be shown in a popup window.
        - script: <target> [args...]

            If <target> is a file path, it will be executed directly (or by Python when
            it ends with ".py"). If <target> looks like a Python module path, it will be
            executed via ``python -m <target>``.

            The popup streams stdout/stderr output and auto-closes when the process exits.
            Closing the popup requests an interrupt (SIGINT).

        - cmd: <cmd> [args...]

            Runs <cmd> from the current environment PATH and streams its output.
            Closing the popup requests an interrupt (SIGINT).
"""
OnPress = Callable[[ButtonName], None]


_PYTHON_MODULE_RE = re.compile(r"^[A-Za-z_]\w*(\.[A-Za-z_]\w*)*$")


def _looks_like_module_path(value: str) -> bool:
    """Return True if value looks like a Python module path."""

    return bool(_PYTHON_MODULE_RE.match(value))


class ButtonUILayout(BaseModel, frozen=True):
    """Button panel UI layout."""

    rows: Optional[List[List[ButtonName]]] = None
    """Optional 2D button grid; use "" for blank cells."""
    n_rows: Optional[int] = None
    """Row count used when inferring layout (optional)."""
    n_cols: Optional[int] = None
    """Column count used when inferring layout (optional)."""
    title: Optional[str] = None
    """Window title (optional)."""
    button_width: int = 10
    """Button width in Tk character units."""
    button_height: int = 2
    """Button height in Tk text lines."""
    padx: int = 4
    """Horizontal padding for each grid cell."""
    pady: int = 4
    """Vertical padding for each grid cell."""
    sticky: str = "nsew"
    """Tk grid sticky option, e.g. "nsew"."""

    def resolve_rows(self, buttons: Sequence[str]) -> List[List[str]]:
        """Resolve the final 2D grid based on rows or inferred layout."""
        if self.rows is not None:
            return [list(r) for r in self.rows]

        buttons = [b.strip() for b in buttons if (b or "").strip()]
        if not buttons:
            return []

        n_rows = self.n_rows
        n_cols = self.n_cols

        if n_rows is None and n_cols is None:
            # Default: single row.
            n_cols = len(buttons)

        if n_cols is None:
            assert n_rows is not None
            if n_rows <= 0:
                raise ValueError("n_rows must be > 0")
            n_cols = int(ceil(len(buttons) / n_rows))
        if n_cols <= 0:
            raise ValueError("n_cols must be > 0")

        rows: List[List[str]] = []
        cur: List[str] = []
        for name in buttons:
            cur.append(name)
            if len(cur) >= n_cols:
                rows.append(cur)
                cur = []
        if cur:
            rows.append(cur)

        # Pad rows so the grid remains rectangular.
        for r in rows:
            if len(r) < n_cols:
                r.extend([""] * (n_cols - len(r)))

        if n_rows is not None and len(rows) < n_rows:
            # Add empty rows if requested.
            rows.extend([[""] * n_cols for _ in range(n_rows - len(rows))])

        return rows

    def iter_buttons(self, buttons: Sequence[str]) -> List[str]:
        """Flatten resolved rows into a list of non-empty button names."""
        resolved = self.resolve_rows(buttons)
        flat: List[str] = []
        for row in resolved:
            for key in row:
                key = (key or "").strip()
                if key:
                    flat.append(key)
        return flat


class TkButtonPanelConfig(BaseModel):
    """Configuration for a Tkinter button panel listener."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    layout: ButtonUILayout = Field(default_factory=ButtonUILayout)
    """Layout settings for the button panel."""
    buttons: List[ButtonName] = Field(default_factory=list)
    """Optional explicit order for buttons when layout.rows is not provided.
    If empty, it defaults to the insertion order of button_callbacks."""
    button_callbacks: Dict[ButtonName, ButtonCallback] = Field(default_factory=dict)
    """
    Optional mapping: button name -> callback for that specific button.
    This is intended for programmatic usage (not hydra-yaml), since callables
    are not serializable.
    """
    on_press: Optional[OnPress] = None
    """Optional unified callback: will be called on any button press."""
    on_close: Optional[Callable[[], None]] = None
    """Callback triggered when the window is closed via the close button."""

    @model_validator(mode="after")
    def _validate_callbacks(self):
        """Validate button layout and callback configuration."""
        ordered_buttons = self.buttons or list(self.button_callbacks.keys())
        layout_buttons = set(self.layout.iter_buttons(ordered_buttons))

        if self.layout.rows is None and not ordered_buttons:
            raise ValueError(
                "layout.rows is not set; provide buttons or button_callbacks to infer layout"
            )

        if self.on_press is None and not self.button_callbacks:
            raise ValueError("Either on_press or button_callbacks must be configured")

        unknown_callback_buttons = set(self.button_callbacks.keys()) - layout_buttons
        if unknown_callback_buttons:
            raise ValueError(
                "button_callbacks contains buttons not present in layout: "
                f"{sorted(unknown_callback_buttons)}"
            )

        if self.on_press is None:
            missing = layout_buttons - set(self.button_callbacks.keys())
            if missing:
                raise ValueError(
                    "Missing per-button callbacks for buttons (on_press is not set): "
                    f"{sorted(missing)}"
                )

        for button_name, callback in self.button_callbacks.items():
            if isinstance(callback, str) and callback.strip().startswith("script:"):
                script_target = callback.split(":", 1)[1].strip()
                if not script_target:
                    raise ValueError(
                        f"script callback for button '{button_name}' is empty"
                    )

                parts = shlex.split(script_target)
                if not parts:
                    raise ValueError(
                        f"script callback for button '{button_name}' is empty"
                    )
                target = parts[0]
                path_candidate = Path(target).expanduser()
                is_path_like = (
                    "/" in target
                    or "\\" in target
                    or target.endswith(".py")
                    or path_candidate.exists()
                )
                if is_path_like:
                    if not path_candidate.exists():
                        raise ValueError(
                            f"script callback for button '{button_name}' not found: {path_candidate}"
                        )
                else:
                    if not _looks_like_module_path(target):
                        raise ValueError(
                            f"script callback for button '{button_name}' is not a valid module path: {target}"
                        )

            if isinstance(callback, str) and callback.strip().startswith("cmd:"):
                cmd_target = callback.split(":", 1)[1].strip()
                if not cmd_target:
                    raise ValueError(
                        f"cmd callback for button '{button_name}' is empty"
                    )
                parts = shlex.split(cmd_target)
                if not parts:
                    raise ValueError(
                        f"cmd callback for button '{button_name}' is empty"
                    )
                cmd_name = parts[0]
                if shutil.which(cmd_name) is None:
                    raise ValueError(
                        f"cmd callback for button '{button_name}' not found in PATH: {cmd_name}"
                    )

        return self


class Listener:
    """A Tkinter-based listener that triggers callbacks on mouse clicks."""

    def __init__(self, config: TkButtonPanelConfig) -> None:
        """Create a listener.

        Args:
            config: Button panel configuration.
        """
        self._config = config
        self._root: Optional[tk.Tk] = None
        self._close_callback_called = False

    def start(self) -> Self:
        """Create the Tk window and build the UI."""
        if self._root is not None:
            return self

        self._root = tk.Tk()
        if self._config.layout.title:
            self._root.title(self._config.layout.title)

        # Closing the window should stop and destroy cleanly.
        self._root.protocol("WM_DELETE_WINDOW", self._handle_window_close)

        self._build_ui(self._root)
        return self

    def stop(self) -> None:
        """Destroy the window if it exists."""
        if self._root is None:
            return

        root = self._root
        self._root = None
        try:
            root.destroy()
        except Exception:
            pass

    def update(self) -> bool:
        """Pump Tk events.

        Returns:
            True if the window is still alive, False if it is closed/stopped.
        """

        if self._root is None:
            return False
        try:
            self._root.update_idletasks()
            self._root.update()
            return True
        except tk.TclError:
            # Usually means the window has been closed.
            self._root = None
            return False

    def join(self, timeout: Optional[float] = None) -> None:
        """Block until window is closed.

        This is mainly for simple scripts. In an app with its own main loop,
        prefer calling update() periodically.
        """

        deadline = None if timeout is None else (time.monotonic() + timeout)
        while True:
            alive = self.update()
            if not alive:
                return
            if deadline is not None and time.monotonic() >= deadline:
                return
            time.sleep(0.01)

    def _build_ui(self, root: tk.Tk) -> None:
        """Build the button grid UI."""
        container = tk.Frame(root)
        container.grid(row=0, column=0, sticky="nsew")

        # Make window resizable.
        root.grid_rowconfigure(0, weight=1)
        root.grid_columnconfigure(0, weight=1)

        ordered_buttons = self._config.buttons or list(
            self._config.button_callbacks.keys()
        )
        rows = self._config.layout.resolve_rows(ordered_buttons)

        n_rows = len(rows)
        n_cols = max((len(r) for r in rows), default=0)

        for r in range(n_rows):
            container.grid_rowconfigure(r, weight=1)
        for c in range(n_cols):
            container.grid_columnconfigure(c, weight=1)

        for r, row in enumerate(rows):
            for c, key in enumerate(row):
                key = (key or "").strip()
                if not key:
                    spacer = tk.Label(container, text="")
                    spacer.grid(
                        row=r,
                        column=c,
                        padx=self._config.layout.padx,
                        pady=self._config.layout.pady,
                        sticky=self._config.layout.sticky,
                    )
                    continue

                btn = tk.Button(
                    container,
                    text=key,
                    width=self._config.layout.button_width,
                    height=self._config.layout.button_height,
                    command=lambda k=key: self._emit(k),
                )
                btn.grid(
                    row=r,
                    column=c,
                    padx=self._config.layout.padx,
                    pady=self._config.layout.pady,
                    sticky=self._config.layout.sticky,
                )

    def _emit(self, key: str) -> None:
        """Dispatch callbacks for a pressed button."""
        cb = self._config.button_callbacks.get(key)
        if cb is not None:
            if isinstance(cb, str):
                self._handle_string_callback(cb)
            else:
                cb()
        if self._config.on_press is not None:
            self._config.on_press(key)

    def _handle_string_callback(self, value: str) -> None:
        """Handle string callbacks.

        Formats:
        - Plain string: show a popup window with the string.
        - "script: <target> [args...]": run a file path or python module and stream output.
        - "cmd: <cmd> [args...]": run a command from PATH and stream output.
        """

        value = value.strip()
        if value.startswith("script:"):
            script_path_str = value.split(":", 1)[1].strip()
            self._run_script_popup(script_path_str)
            return
        if value.startswith("cmd:"):
            cmd_str = value.split(":", 1)[1].strip()
            self._run_cmd_popup(cmd_str)
            return
        self._show_text_popup(value)

    def _show_text_popup(self, text: str) -> None:
        """Show a small popup window displaying the given text."""
        if self._root is None:
            return

        popup = tk.Toplevel(self._root)
        popup.transient(self._root)

        msg = tk.Message(popup, text=text, width=600)
        msg.pack(padx=12, pady=12)

        close_btn = tk.Button(popup, text="close", command=popup.destroy)
        close_btn.pack(padx=12, pady=(0, 12))

    def _run_script_popup(self, script_path_str: str) -> None:
        """Run a script and show its output in a popup.

        The popup auto-closes when the script exits.
        Closing the popup sends an interrupt request (SIGINT).
        """

        if self._root is None:
            return

        parts = shlex.split(script_path_str)
        if not parts:
            self._show_text_popup("script callback is empty")
            return

        target, *args = parts
        path_candidate = Path(target).expanduser()
        is_path_like = (
            "/" in target
            or "\\" in target
            or target.endswith(".py")
            or path_candidate.exists()
        )

        if is_path_like:
            if path_candidate.suffix == ".py":
                cmd = [sys.executable, str(path_candidate), *args]
            else:
                cmd = [str(path_candidate), *args]
        else:
            # Treat as python module path.
            cmd = [sys.executable, "-m", target, *args]

        self._run_process_popup(cmd=cmd, title=f"script: {script_path_str}")

    def _run_cmd_popup(self, cmd_str: str) -> None:
        """Run a command and show its output in a popup.

        The popup auto-closes when the command exits.
        Closing the popup sends an interrupt request (SIGINT).
        """

        if self._root is None:
            return

        parts = shlex.split(cmd_str)
        if not parts:
            self._show_text_popup("cmd callback is empty")
            return

        cmd_name = parts[0]
        if shutil.which(cmd_name) is None:
            self._show_text_popup(f"cmd not found in PATH: {cmd_name}")
            return

        self._run_process_popup(cmd=parts, title=f"cmd: {cmd_str}")

    def _run_process_popup(self, cmd: List[str], title: str) -> None:
        """Run a subprocess and stream its output to a popup."""

        if self._root is None:
            return

        popup = tk.Toplevel(self._root)
        popup.transient(self._root)
        popup.title(title)

        text = tk.Text(popup, wrap="word")
        scrollbar = tk.Scrollbar(popup, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)

        text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            popup.destroy()
            self._show_text_popup(f"Failed to start process: {' '.join(cmd)}\n{exc}")
            return

        output_queue: Queue[str | None] = Queue()
        stop_requested = {"count": 0}

        def request_interrupt() -> None:
            stop_requested["count"] += 1

            if proc.poll() is not None:
                try:
                    popup.destroy()
                except Exception:
                    pass
                return

            if stop_requested["count"] == 1:
                try:
                    proc.send_signal(signal.SIGINT)
                except Exception:
                    pass
                popup.title("interrupt requested")

                def escalate_to_terminate() -> None:
                    if proc.poll() is None:
                        try:
                            proc.terminate()
                        except Exception:
                            pass

                def escalate_to_kill() -> None:
                    if proc.poll() is None:
                        try:
                            proc.kill()
                        except Exception:
                            pass

                popup.after(2000, escalate_to_terminate)
                popup.after(4000, escalate_to_kill)
                return

            # Second close attempt: force stop.
            try:
                proc.terminate()
            except Exception:
                pass

        popup.protocol("WM_DELETE_WINDOW", request_interrupt)

        def reader() -> None:
            try:
                assert proc.stdout is not None
                for line in proc.stdout:
                    output_queue.put(line)
            finally:
                output_queue.put(None)

        threading.Thread(target=reader, daemon=True).start()

        def pump() -> None:
            try:
                while True:
                    item = output_queue.get_nowait()
                    if item is None:
                        try:
                            popup.destroy()
                        except Exception:
                            pass
                        return
                    text.insert("end", item)
                    text.see("end")
            except Exception:
                pass

            if popup.winfo_exists():
                popup.after(50, pump)

        popup.after(50, pump)

    def _handle_window_close(self) -> None:
        """Handle the window manager close action."""
        if not self._close_callback_called:
            self._close_callback_called = True
            if self._config.on_close is not None:
                self._config.on_close()
        self.stop()


def infer_rows(keys: Sequence[str], *, n_cols: int) -> List[List[str]]:
    """Convert a flat button list to a row/col layout."""

    if n_cols <= 0:
        raise ValueError("n_cols must be > 0")

    rows: List[List[str]] = []
    cur: List[str] = []
    for key in keys:
        cur.append(key)
        if len(cur) >= n_cols:
            rows.append(cur)
            cur = []
    if cur:
        rows.append(cur)
    return rows
