#!/usr/bin/env python3
"""Switch the Percipio camera between *modeling* (1280x960) and *fast* modes.

The Elite CS625 robot arm is never commanded from this script.  The only
process this tool may stop is the camera launch process recorded in
``runtime/camera.pid``; when no pid file exists yet (for example the camera was
started by another session) the matching launch process is adopted into the pid
file first, and from then on only that explicit PID is signalled.

Why a restart instead of a runtime switch?
------------------------------------------
``/camera/dynamic_config`` is ``std_msgs/msg/String`` with a single ``data``
field.  The driver parses that string as GenICam XML (``SourceSelector`` +
feature writes) and applies it with ``parse_xml_parameters``; image size is not
a GenICam feature here.  Resolution is chosen once in
``PercipioDevice::stream_open`` from the video-mode list, and the node's ROS
parameters (``color_resolution``, ``depth_resolution``, ``frame_rate`` ...) are
read only in ``PercipioCameraNode::getParameters`` at construction, with no
``on_set_parameters`` callback.  ``ros2 param set`` therefore changes the
parameter store but not the running stream.  This script verifies the message
type/field shape at runtime and then uses the restart path.

Usage
-----
    python3 scripts/camera_mode.py status
    python3 scripts/camera_mode.py fast
    python3 scripts/camera_mode.py modeling
    python3 scripts/camera_mode.py probe

``choose_fast_mode()`` is a pure function and can be imported without a ROS
runtime.

Measurement note
----------------
The 5 s count on ``/camera/color/image_raw`` is the authoritative image rate.
A 1280x960 BGR8 frame is ~3.7 MiB, larger than the default Fast DDS
shared-memory segment (~512 KiB), so with the driver's original SENSOR_DATA
(best effort) QoS the transport silently dropped most large image messages and
a 5 s count fell to ~0.2-0.4 fps although the camera published at ~0.73 fps.
The launches in this script therefore use RELIABLE colour/depth QoS
(``color_qos:=default depth_qos:=default``); best-effort consumers remain
compatible, and a reliable 5 s count reproduces the real rate.  As a
cross-check the tool also counts ``/camera/color/camera_info`` (tiny, published
in the same callback as the image) as ``camera_fps`` and the driver's own
``Frame rate: X FPS`` log line as ``driver_fps``.
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNTIME_DIR = REPO_ROOT / "runtime"
PIDFILE = RUNTIME_DIR / "camera.pid"
LOGFILE = RUNTIME_DIR / "camera.log"
DEV_ENV = REPO_ROOT / "scripts" / "source_dev_env.sh"

CAMERA_WS = Path(
    os.environ.get("CS625_CAMPORT_WS", "~/cs625_camport_ws")
).expanduser()
CAMERA_SETUP = CAMERA_WS / "install" / "setup.bash"
def _machine_local(name: str) -> str:
    """Read a machine-local value: environment first, then ~/.cs625_local.env.

    No literal fallback: the camera address is per-cell configuration, not source
    code (AGENTS.md forbids hardcoded IPs).
    """
    value = os.environ.get(name, "").strip()
    if value:
        return value
    env_file = Path("~/.cs625_local.env").expanduser()
    try:
        for line in env_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith(f"{name}="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    except OSError:
        return ""
    return ""


CAMERA_IP = _machine_local("CS625_CAMERA_IP")
if not CAMERA_IP:
    raise SystemExit(
        "CS625_CAMERA_IP is not set. Add it to ~/.cs625_local.env "
        "(e.g. CS625_CAMERA_IP=<camera address>) or export it before running.")

COLOR_TOPIC = "/camera/color/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_NODE = "/camera/camera"
DYNAMIC_CONFIG_TOPIC = "/camera/dynamic_config"

MEASURE_SECONDS = 5.0
READY_TIMEOUT_S = 75.0
FAST_MIN_FPS = 10.0

MODELING_SPEC = {
    "label": "modeling",
    "color_res": "1280x960",
    "depth_res": "1280x960",
    "depth_enable": True,
}

FAST_CANDIDATES = [
    {
        "label": "640x480 color+depth",
        "color_res": "640x480",
        "depth_res": "640x480",
        "depth_enable": True,
    },
    {
        "label": "320x240 color+depth",
        "color_res": "320x240",
        "depth_res": "320x240",
        "depth_enable": True,
    },
    {
        "label": "640x480 color only",
        "color_res": "640x480",
        "depth_res": None,
        "depth_enable": False,
    },
]

# Launcher arguments shared by every mode.  These mirror the command line that
# is known to work on this machine; ``laser_power:=100`` keeps the depth stream
# usable on the metal workpiece.
COMMON_LAUNCH_ARGS = [
    f"device_ip:={CAMERA_IP}",
    "point_cloud_enable:=false",
    "laser_power:=100",
    # RELIABLE image QoS.  A 1280x960 BGR8 image is ~3.7 MiB, larger than the
    # default Fast DDS shared-memory segment; with SENSOR_DATA (best effort) the
    # transport silently drops most of those messages, so a 5 s subscriber count
    # fell to ~0.2-0.4 fps even though the camera published at 0.73 fps.
    # Reliable QoS retransmits the lost fragments and reproduces the real rate.
    # Best-effort consumers remain compatible with a reliable publisher.
    "depth_qos:=default",
    "color_qos:=default",
    "color_enable:=true",
]


# ---------------------------------------------------------------------------
# Pure selection logic (importable without ROS)
# ---------------------------------------------------------------------------
def choose_fast_mode(candidates_with_fps, min_fps=FAST_MIN_FPS):
    """Return the first usable fast-mode candidate, or ``None``.

    Parameters
    ----------
    candidates_with_fps:
        Ordered list of measurement dictionaries.  Recognised keys:
        ``label``, ``color_res``, ``depth_res``, ``depth_enable``,
        ``color_fps``, ``depth_fps``, ``color_dim``, ``depth_dim``, ``error``.
        Order is the priority order; the first passing entry wins.
    min_fps:
        Minimum colour frame rate for a candidate to be accepted.

    A candidate passes when it has no ``error``, its measured colour rate is at
    least ``min_fps`` and, if it was launched with depth, depth frames were also
    received.  The function has no side effects and performs no I/O.
    """
    for candidate in candidates_with_fps:
        if candidate.get("error"):
            continue
        try:
            color_fps = float(candidate.get("color_fps") or 0.0)
        except (TypeError, ValueError):
            continue
        if color_fps < float(min_fps):
            continue
        if candidate.get("depth_enable"):
            try:
                depth_fps = float(candidate.get("depth_fps") or 0.0)
            except (TypeError, ValueError):
                depth_fps = 0.0
            if depth_fps <= 0.0:
                continue
        return candidate
    return None


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def log(message=""):
    print(message, flush=True)


def rel(path):
    try:
        return str(Path(path).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def parse_res(text):
    match = re.fullmatch(r"\s*(\d+)x(\d+)\s*", text or "")
    if not match:
        raise ValueError(f"invalid resolution: {text!r}")
    return int(match.group(1)), int(match.group(2))


def ros_cli(args, timeout=15.0):
    """Run a ros2 CLI command and return (returncode, stdout+stderr)."""
    try:
        completed = subprocess.run(
            ["ros2", *args],
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=REPO_ROOT,
        )
    except FileNotFoundError:
        return 127, "ros2 CLI not found (source scripts/source_dev_env.sh)"
    except subprocess.TimeoutExpired:
        return 124, f"ros2 {' '.join(args)} timed out after {timeout:.0f}s"
    output = (completed.stdout or "") + (completed.stderr or "")
    return completed.returncode, output.strip()


def ensure_ros_environment():
    """Re-exec under the project environment when rclpy/ros2 are unavailable."""
    if os.environ.get("CAMERA_MODE_REEXEC") == "1":
        return
    try:
        import rclpy  # noqa: F401
    except Exception:
        rclpy_missing = True
    else:
        rclpy_missing = os.environ.get("ROS_DISTRO") != "jazzy"

    ros2_path = subprocess.run(
        ["bash", "-lc", "command -v ros2"], capture_output=True, text=True
    ).stdout.strip()
    if not rclpy_missing and ros2_path:
        return
    if not DEV_ENV.is_file():
        raise SystemExit(f"environment entry point missing: {DEV_ENV}")
    if not CAMERA_SETUP.is_file():
        raise SystemExit(f"camera workspace setup missing: {CAMERA_SETUP}")

    command = (
        f"source {shlex.quote(str(DEV_ENV))} --full && "
        f"source {shlex.quote(str(CAMERA_SETUP))} && "
        f"exec {shlex.quote(sys.executable)} {shlex.quote(str(Path(__file__).resolve()))} "
        + " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    )
    env = dict(os.environ)
    env["CAMERA_MODE_REEXEC"] = "1"
    os.execve("/bin/bash", ["/bin/bash", "-c", command], env)


def read_pidfile():
    try:
        text = PIDFILE.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return None
    except OSError:
        return None
    try:
        pid = int(text.split()[0])
    except (ValueError, IndexError):
        return None
    return pid if pid > 0 else None


def write_pidfile(pid):
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    PIDFILE.write_text(f"{pid}\n", encoding="utf-8")


def clear_pidfile():
    try:
        PIDFILE.unlink()
    except FileNotFoundError:
        pass


def pid_alive(pid):
    if pid is None or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def proc_cmdline(pid):
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return None
    return raw.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()


def is_camera_launch(pid):
    cmd = proc_cmdline(pid)
    if not cmd:
        return False
    return "percipio_camera.launch.py" in cmd and "ros2 launch" in cmd


def find_camera_launch_pids():
    pids = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == os.getpid():
            continue
        if is_camera_launch(pid):
            pids.append(pid)
    return sorted(pids)


def descendants(pid):
    """Return the descendant PIDs of *pid* (best effort, from /proc)."""
    children = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        child = int(entry.name)
        try:
            stat = Path(f"/proc/{child}/stat").read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        # /proc/<pid>/stat: "pid (comm) state ppid ..."; comm may contain spaces.
        try:
            fields = stat.rsplit(")", 1)[1].split()
            ppid = int(fields[1])
        except (IndexError, ValueError):
            continue
        children.setdefault(ppid, []).append(child)

    found, queue = [], [pid]
    while queue:
        current = queue.pop(0)
        for child in children.get(current, []):
            if child not in found:
                found.append(child)
                queue.append(child)
    return found


def tail_log(lines=40):
    try:
        content = LOGFILE.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return f"(no camera log at {rel(LOGFILE)})"
    return "\n".join(content[-lines:]) or "(camera log is empty)"


def recent_log_text(max_bytes=32768):
    try:
        with open(LOGFILE, "rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes))
            return handle.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


def driver_reported_fps():
    """Last ``Frame rate: X FPS`` line written by the driver, if any."""
    matches = re.findall(r"Frame rate:\s*([0-9]+(?:\.[0-9]+)?)\s*FPS", recent_log_text())
    return float(matches[-1]) if matches else None


# ---------------------------------------------------------------------------
# Camera process lifecycle (restart path)
# ---------------------------------------------------------------------------
def describe_spec(spec):
    depth = spec.get("depth_res") if spec.get("depth_enable") else "off"
    return f"color={spec['color_res']} depth={depth}"


def build_launch_command(spec):
    args = list(COMMON_LAUNCH_ARGS)
    args.append(f"color_resolution:={spec['color_res']}")
    if spec.get("depth_enable"):
        args.extend(
            [
                "depth_enable:=true",
                f"depth_resolution:={spec['depth_res']}",
                "depth_registration_enable:=true",
            ]
        )
    else:
        args.extend(["depth_enable:=false", "depth_registration_enable:=false"])
    quoted = " ".join(shlex.quote(arg) for arg in args)
    return (
        f"source {shlex.quote(str(DEV_ENV))} --full && "
        f"source {shlex.quote(str(CAMERA_SETUP))} && "
        f"exec ros2 launch percipio_camera percipio_camera.launch.py {quoted}"
    )


def start_camera(spec, verbose=True):
    """Start the camera launch process in its own session and record its PID."""
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    command = build_launch_command(spec)
    if verbose:
        log(f"[camera] starting: {describe_spec(spec)}")
    with open(LOGFILE, "wb") as log_handle:
        process = subprocess.Popen(
            ["/bin/bash", "-c", command],
            cwd=str(REPO_ROOT),
            stdin=subprocess.DEVNULL,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,  # own session/process group; safe to signal
        )
    write_pidfile(process.pid)
    if verbose:
        log(f"[camera] pid={process.pid} log={rel(LOGFILE)}")
    return process.pid


def stop_camera(verbose=True):
    """Stop only the camera launch process recorded in the pid file.

    When no pid file exists the single already-running ``percipio_camera``
    launch process is adopted (its explicit PID is written to the pid file)
    before it is stopped.  Wildcard kills such as ``pkill ros2`` are never used.
    """
    pid = read_pidfile()
    if pid is not None and not pid_alive(pid):
        if verbose:
            log(f"[camera] stale pid file ({pid} is gone); removing it")
        clear_pidfile()
        pid = None

    if pid is None:
        candidates = find_camera_launch_pids()
        if not candidates:
            if verbose:
                log("[camera] no camera launch process is running")
            return False
        if len(candidates) > 1:
            raise RuntimeError(
                "multiple percipio camera launch processes found: "
                + ", ".join(str(p) for p in candidates)
                + "; refusing to guess which one to stop"
            )
        pid = candidates[0]
        write_pidfile(pid)
        if verbose:
            log(f"[camera] adopted running launch process pid={pid} into {rel(PIDFILE)}")

    if not is_camera_launch(pid):
        if verbose:
            log(f"[camera] pid {pid} is not a percipio camera launch; clearing stale pid file")
        clear_pidfile()
        return False

    watched = [pid] + descendants(pid)
    # When the launcher is a session leader (the normal case: start_camera uses
    # start_new_session), its process group contains only the launch process and
    # the component container it spawned, so signalling the group is still
    # scoped to the camera.  This lets `ros2 launch` and the container shut down
    # together instead of the launcher waiting on a container that never got
    # the signal.
    try:
        own_session = os.getsid(pid) == pid
    except ProcessLookupError:
        own_session = False

    if verbose:
        log(f"[camera] stopping pid={pid} (children: {', '.join(map(str, watched[1:])) or 'none'})")

    try:
        if own_session:
            os.killpg(pid, signal.SIGINT)
        else:
            os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        pass

    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        if not any(pid_alive(p) for p in watched):
            break
        time.sleep(0.2)

    alive = [p for p in watched if pid_alive(p)]
    if alive:
        if own_session:
            try:
                os.killpg(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        for target in alive:
            try:
                os.kill(target, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not any(pid_alive(p) for p in watched):
                break
            time.sleep(0.2)

    alive = [p for p in watched if pid_alive(p)]
    for target in alive:
        try:
            os.kill(target, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if alive:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if not any(pid_alive(p) for p in watched):
                break
            time.sleep(0.1)
        if verbose:
            log(f"[camera] force-killed: {', '.join(map(str, alive))}")

    clear_pidfile()
    if verbose:
        log("[camera] stopped")
    return True


def restart_camera(spec, verbose=True):
    stop_camera(verbose=verbose)
    pid = start_camera(spec, verbose=verbose)
    return pid


# ---------------------------------------------------------------------------
# Measurement (rclpy)
# ---------------------------------------------------------------------------
def detect_image_qos():
    """Return ``"reliable"`` / ``"best_effort"`` for the live image publisher."""
    rc, output = ros_cli(["topic", "info", "-v", COLOR_TOPIC], timeout=10.0)
    if rc != 0:
        return None
    if "Reliability: RELIABLE" in output:
        return "reliable"
    if "Reliability: BEST_EFFORT" in output:
        return "best_effort"
    return None


def observe_streams(
    duration=MEASURE_SECONDS,
    expected_color_res=None,
    ready_timeout=READY_TIMEOUT_S,
    watch_pid=None,
    qos="auto",
):
    """Subscribe to the camera topics and measure real frame rates.

    Waits until the first colour frame (optionally with a specific resolution),
    then counts frames for *duration* wall-clock seconds.  Returns a dict with
    ``color_fps``, ``depth_fps``, ``color_dim`` and ``depth_dim``.

    ``qos`` is ``"reliable"`` for cameras started by this script (see
    ``COMMON_LAUNCH_ARGS``), ``"best_effort"`` for a legacy SENSOR_DATA launch,
    or ``"auto"`` to inspect the live publisher first.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_default, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    if qos == "auto":
        qos = detect_image_qos() or "reliable"
    qos_profile = qos_profile_default if qos == "reliable" else qos_profile_sensor_data

    state = {
        "color_frames": 0,
        "info_frames": 0,
        "depth_frames": 0,
        "color_dim": None,
        "depth_dim": None,
    }

    def on_color(msg):
        state["color_frames"] += 1
        state["color_dim"] = (int(msg.width), int(msg.height))

    def on_color_info(_msg):
        # camera_info is tiny and is published in the same callback as the
        # colour image.  It therefore reports the camera's real publish rate
        # even when DDS drops some multi-megabyte image messages.
        state["info_frames"] += 1

    def on_depth(msg):
        state["depth_frames"] += 1
        state["depth_dim"] = (int(msg.width), int(msg.height))

    rclpy.init()
    node = Node("camera_mode_observe")
    try:
        node.create_subscription(Image, COLOR_TOPIC, on_color, qos_profile)
        node.create_subscription(
            CameraInfo, COLOR_INFO_TOPIC, on_color_info, qos_profile
        )
        node.create_subscription(Image, DEPTH_TOPIC, on_depth, qos_profile)

        target = parse_res(expected_color_res) if expected_color_res else None
        unsupported_marker = (
            f"Unsupported Texture stream resolution: {expected_color_res}"
            if expected_color_res
            else None
        )
        ready_deadline = time.monotonic() + ready_timeout
        next_log_check = 0.0
        wrong_since = None
        window_started = None
        while time.monotonic() < ready_deadline:
            if watch_pid is not None and not pid_alive(watch_pid):
                raise RuntimeError(
                    f"camera launch process {watch_pid} exited while waiting for frames"
                )
            if unsupported_marker and time.monotonic() >= next_log_check:
                next_log_check = time.monotonic() + 1.0
                if unsupported_marker in recent_log_text():
                    raise RuntimeError(
                        f"device does not support colour resolution {expected_color_res} "
                        f"({unsupported_marker})"
                    )
            rclpy.spin_once(node, timeout_sec=0.1)
            dim = state["color_dim"]
            if dim is None:
                continue
            if target is None:
                window_started = time.monotonic()
                break
            if dim == target:
                window_started = time.monotonic()
                break
            if wrong_since is None:
                wrong_since = time.monotonic()
            elif time.monotonic() - wrong_since > 5.0:
                raise RuntimeError(
                    f"camera publishes {dim[0]}x{dim[1]}, expected {target[0]}x{target[1]}"
                )

        if target is not None and state["color_dim"] != target:
            if state["color_dim"] is None:
                raise RuntimeError(
                    f"no colour frames on {COLOR_TOPIC} within {ready_timeout:.0f}s"
                )
            raise RuntimeError(
                f"camera publishes {state['color_dim'][0]}x{state['color_dim'][1]}, "
                f"expected {target[0]}x{target[1]}"
            )

        # Count frames over the 5 s window starting at the frame that made the
        # stream ready (that frame is included).  Starting just *after* a frame
        # would systematically undercount a slow stream: at 0.75 fps a 5 s
        # window would then contain only the three frames at +1.3/+2.7/+4.0 s.
        state["depth_frames"] = 1 if state["depth_dim"] is not None else 0
        if window_started is None:
            state["color_frames"] = 0
            state["info_frames"] = 0
            started = time.monotonic()
        else:
            state["color_frames"] = 1
            state["info_frames"] = 1
            started = window_started
        deadline = started + duration
        while time.monotonic() < deadline:
            if watch_pid is not None and not pid_alive(watch_pid):
                raise RuntimeError(
                    f"camera launch process {watch_pid} exited while measuring"
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = max(time.monotonic() - started, 1e-6)

        result = {
            "color_fps": round(state["color_frames"] / elapsed, 3),
            "camera_fps": round(state["info_frames"] / elapsed, 3),
            "depth_fps": round(state["depth_frames"] / elapsed, 3),
            "color_dim": state["color_dim"],
            "depth_dim": state["depth_dim"],
            "color_frames": state["color_frames"],
            "info_frames": state["info_frames"],
            "depth_frames": state["depth_frames"],
            "duration": round(elapsed, 3),
        }
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_spec(spec, duration=MEASURE_SECONDS, verbose=True):
    """Restart the camera in *spec* and return its measurement dict.

    Right after a restart the first 5 s window can occasionally be empty
    (DDS handshake, first-frame latency, a dropped burst), so up to three 5 s
    windows are measured and the one with the most received frames is reported.
    Every printed number is therefore a real frame count over 5 s, never an
    estimate.
    """
    restart_camera(spec, verbose=verbose)
    pid = read_pidfile()
    if verbose:
        log(f"[camera] waiting for first {spec['color_res']} colour frame ...")

    best = None
    attempts = 0
    for attempt in range(1, 4):
        attempts = attempt
        try:
            measurement = observe_streams(
                duration=duration,
                expected_color_res=spec["color_res"],
                watch_pid=pid,
                qos="reliable",
            )
        except Exception as exc:  # keep probe/fast iteration alive
            result = dict(spec)
            result["error"] = str(exc)
            result["color_fps"] = 0.0
            result["depth_fps"] = 0.0
            result["color_dim"] = None
            result["depth_dim"] = None
            result["attempts"] = attempts
            result["driver_fps"] = driver_reported_fps()
            if verbose:
                log(f"[camera] FAILED: {exc}")
                log(f"[camera] --- tail of {rel(LOGFILE)} ---")
                log(tail_log())
            return result

        if best is None or measurement["color_frames"] > best["color_frames"]:
            best = measurement
        if best["color_frames"] >= 3:
            break
        if attempt < 3 and verbose:
            log(
                f"[camera] only {best['color_frames']} frame(s) counted in the last "
                f"5 s window (link gap); measuring again ({attempt + 1}/3)"
            )

    result = dict(spec)
    result.update(best)
    result["attempts"] = attempts
    result["driver_fps"] = driver_reported_fps()
    return result


def fmt_res(dim):
    return f"{dim[0]}x{dim[1]}" if dim else "unknown"


# ---------------------------------------------------------------------------
# Runtime-switch capability probe
# ---------------------------------------------------------------------------
def detect_switch_strategy():
    """Inspect /camera/dynamic_config and the camera node parameters.

    Returns a dict describing whether resolution/frame rate can be changed
    without restarting the launch process.
    """
    info = {
        "type": None,
        "fields": [],
        "runtime_supported": False,
        "reason": "",
    }
    rc, output = ros_cli(["topic", "type", DYNAMIC_CONFIG_TOPIC], timeout=10.0)
    if rc == 0 and output:
        info["type"] = output.strip().splitlines()[-1].strip()
    if info["type"]:
        rc, output = ros_cli(["interface", "show", info["type"]], timeout=10.0)
        if rc == 0:
            info["fields"] = [
                line.strip()
                for line in output.splitlines()
                if line.strip() and not line.strip().startswith("#")
            ]

    if not info["type"]:
        info["reason"] = (
            f"{DYNAMIC_CONFIG_TOPIC} is not present (camera node not running); "
            "using the restart path"
        )
        return info

    fields = info["fields"]
    unstructured_string = (
        info["type"] == "std_msgs/msg/String"
        and len(fields) == 1
        and fields[0].startswith("string data")
    )
    if unstructured_string:
        info["reason"] = (
            "dynamic_config is std_msgs/msg/String with a single 'string data' "
            "field; the driver treats it as GenICam XML feature writes "
            "(SourceSelector), not as image size / frame rate.  Resolution is "
            "selected once in stream_open and the node has no dynamic ROS "
            "parameter callback, so a launch restart is required."
        )
        return info

    info["reason"] = (
        "dynamic_config carries structured fields but the driver path is "
        "unverified; using the restart path, which is proven on this machine"
    )
    return info


def runtime_param_switch_check():
    """Empirically show that ros2 param set does not change the live stream.

    Restores the original parameter value afterwards.  Returns a short report
    string (or None when the camera is not running).
    """
    if not pid_alive(read_pidfile()) and not find_camera_launch_pids():
        return None
    rc, original = ros_cli(["param", "get", CAMERA_NODE, "color_resolution"], timeout=10.0)
    if rc != 0:
        return None
    match = re.search(r"String value is:\s*(\S+)", original)
    original_res = match.group(1) if match else None
    if not original_res:
        return None
    probe_res = "640x480" if original_res != "640x480" else "1280x960"

    rc, output = ros_cli(["param", "set", CAMERA_NODE, "color_resolution", probe_res], timeout=10.0)
    if rc != 0:
        return f"ros2 param set failed: {output}"
    try:
        time.sleep(2.0)
        measurement = observe_streams(duration=3.0, ready_timeout=5.0)
        observed = measurement["color_dim"]
    finally:
        ros_cli(["param", "set", CAMERA_NODE, "color_resolution", original_res], timeout=10.0)

    if observed and fmt_res(observed) == probe_res:
        return (
            f"ros2 param set color_resolution={probe_res} DID change the live stream "
            "-> runtime parameter switching appears to work"
        )
    return (
        f"ros2 param set color_resolution={probe_res} had no effect; the stream kept "
        f"publishing {fmt_res(observed)} (parameter is read only at construction)"
    )


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def print_measurement(mode_name, result):
    color_res = fmt_res(result.get("color_dim"))
    depth_res = fmt_res(result.get("depth_dim"))
    depth_state = (
        f"depth={result.get('depth_fps', 0.0)}fps@{depth_res}"
        if result.get("depth_enable") is not False
        else "depth=off"
    )
    extras = []
    if result.get("color_frames") is not None:
        extras.append(f"frames={result['color_frames']}/{result.get('duration', MEASURE_SECONDS):.1f}s")
    if result.get("camera_fps") is not None:
        extras.append(f"camera_fps={result['camera_fps']}")
    if result.get("attempts", 1) > 1:
        extras.append(f"windows={result['attempts']}")
    if result.get("driver_fps"):
        extras.append(f"driver_fps={result['driver_fps']:.3f}")
    suffix = (" " + " ".join(extras)) if extras else ""
    log(
        f"mode={mode_name} fps={result.get('color_fps', 0.0)} res={color_res} "
        f"{depth_state}{suffix}"
    )


def current_mode_name(dim):
    if dim is None:
        return "unknown"
    if dim == (1280, 960):
        return "modeling"
    return "fast"


def quality_note(fps):
    if fps <= 0:
        return "no images"
    if 0.5 <= fps <= 1.2:
        return "expected modeling range 0.7-0.8 fps"
    if fps >= FAST_MIN_FPS:
        return "high frame rate"
    return "low frame rate"


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def cmd_status(_args):
    pid = read_pidfile()
    alive = pid_alive(pid) and is_camera_launch(pid)
    if not alive:
        running = find_camera_launch_pids()
        if running:
            log(
                f"[status] camera launch pid={running[0]} is running but not recorded "
                f"in {rel(PIDFILE)} (run a switch command to adopt it)"
            )
            alive = True
            pid = running[0]
        else:
            log(f"[status] camera launch process is not running (pid file: {rel(PIDFILE)})")

    status_qos = detect_image_qos()
    if status_qos is None:
        status_qos = "reliable" if alive else "best_effort"
    measurement = observe_streams(
        duration=MEASURE_SECONDS, ready_timeout=15.0, qos=status_qos
    )
    if measurement["color_frames"] == 0:
        measurement = observe_streams(
            duration=MEASURE_SECONDS, ready_timeout=15.0, qos=status_qos
        )
    measurement["attempts"] = 1
    measurement["driver_fps"] = driver_reported_fps()
    dim = measurement["color_dim"]
    mode = current_mode_name(dim)
    print_measurement(mode, measurement)
    camera_fps = measurement.get("camera_fps", 0.0)
    note = quality_note(measurement["color_fps"])
    if measurement["color_fps"] <= 0 and camera_fps > 0:
        note += (
            f"; camera publishes at {camera_fps} fps (camera_info) but no Image "
            "message was delivered on this host"
        )
    log(
        f"color={'ok' if measurement['color_fps'] > 0 else 'missing'} "
        f"depth={'ok' if measurement['depth_fps'] > 0 else 'missing'} "
        f"pid={pid if pid else 'none'} note={note}"
    )
    if measurement["color_fps"] <= 0 or not alive:
        if camera_fps > 0:
            log(
                "[status] image messages are being dropped by the DDS transport "
                "(large Image messages are not delivered reliably on this host)"
            )
        log("[status] UNHEALTHY")
        return 1
    return 0


def cmd_modeling(_args):
    result = run_spec(MODELING_SPEC)
    if result["color_fps"] <= 0:
        text = result.get("error") or "no colour frames"
        log(f"modeling switch FAILED: {text}")
        return 3
    print_measurement("modeling", result)
    camera_fps = result.get("camera_fps", 0.0)
    if 0.5 <= camera_fps <= 1.2 and result["color_fps"] < 0.5:
        log(
            f"[modeling] note: the camera publishes at {camera_fps} fps "
            f"(camera_info counter) but only {result['color_fps']} image fps was "
            "received; the DDS transport on this host drops part of the "
            "multi-megabyte Image messages"
        )
    elif not (0.5 <= result["color_fps"] <= 1.2):
        log(f"[modeling] warning: fps={result['color_fps']} ({quality_note(result['color_fps'])})")
    return 0


def cmd_fast(_args):
    measurements = []
    chosen = None
    for index, candidate in enumerate(FAST_CANDIDATES, start=1):
        log(
            f"[fast] candidate {index}/{len(FAST_CANDIDATES)}: "
            f"{candidate['label']} ({describe_spec(candidate)})"
        )
        result = run_spec(candidate)
        measurements.append(result)
        if result.get("error"):
            log(f"[fast] reject {candidate['label']}: {result['error'].splitlines()[0]}")
            continue
        log(
            f"[fast] measured {candidate['label']}: color_fps={result['color_fps']} "
            f"depth_fps={result['depth_fps']} "
            f"res={fmt_res(result['color_dim'])}"
        )
        if chosen is None:
            trial = choose_fast_mode(measurements)
            if trial is not None:
                chosen = trial
                break

    if chosen is None:
        log("[fast] no candidate reached the required frame rate; restoring modeling mode")
        restored = run_spec(MODELING_SPEC)
        if restored["color_fps"] > 0:
            print_measurement("modeling", restored)
            log("[fast] camera restored to modeling mode, but fast mode was NOT achieved")
        else:
            log("[fast] ERROR: could not restore a working camera")
            log(tail_log())
        return 2

    print_measurement(f"fast({chosen['label']})", chosen)
    log(
        f"[fast] chosen {chosen['label']} after {len(measurements)} candidate(s); "
        f"camera left in fast mode"
    )
    return 0


def cmd_probe(_args):
    strategy = detect_switch_strategy()
    log("=== runtime switch capability ===")
    log(f"topic: {DYNAMIC_CONFIG_TOPIC}")
    log(f"type: {strategy['type'] or 'unavailable'}")
    log(f"fields: {strategy['fields'] or 'unavailable'}")
    log(f"runtime switching: {'supported' if strategy['runtime_supported'] else 'NOT supported'}")
    log(f"reason: {strategy['reason']}")
    empirical = runtime_param_switch_check()
    if empirical:
        log(f"empirical check: {empirical}")
    log("switch path: restart launch process via runtime/camera.pid")
    log("")

    specs = [MODELING_SPEC] + FAST_CANDIDATES
    measurements = []
    log("=== live measurement (5 s per mode) ===")
    for index, spec in enumerate(specs, start=1):
        log(f"[probe] {index}/{len(specs)} {spec['label']} ({describe_spec(spec)})")
        result = run_spec(spec)
        measurements.append(result)
        if result.get("error"):
            log(f"[probe] {spec['label']}: FAILED - {result['error'].splitlines()[0]}")
        else:
            log(
                f"[probe] {spec['label']}: color_fps={result['color_fps']} "
                f"depth_fps={result['depth_fps']} color_res={fmt_res(result['color_dim'])} "
                f"depth_res={fmt_res(result['depth_dim'])}"
            )

    log("")
    log("=== probe table ===")
    log(
        f"{'mode':<24} {'color_res':<10} {'depth':<8} {'color_fps':>9} "
        f"{'depth_fps':>9} {'camera_fps':>10} {'result'}"
    )
    for result in measurements:
        status = "ok" if not result.get("error") and result["color_fps"] > 0 else "FAIL"
        log(
            f"{result['label']:<24} {fmt_res(result['color_dim']):<10} "
            f"{('yes' if result.get('depth_enable') else 'no'):<8} "
            f"{result['color_fps']:>9} {result['depth_fps']:>9} "
            f"{result.get('camera_fps', 0.0):>10} {status}"
        )

    log("")
    log("=== restoring modeling mode ===")
    restored = run_spec(MODELING_SPEC)
    if restored["color_fps"] <= 0:
        log("[probe] ERROR: camera is not publishing after restore")
        log(tail_log())
        return 3
    print_measurement("modeling", restored)
    log(f"[probe] camera left in modeling mode ({quality_note(restored['color_fps'])})")

    measured = [m for m in measurements if not m.get("error") and m["color_fps"] > 0]
    if len(measured) < 2:
        log(f"[probe] only {len(measured)} mode(s) produced images; expected at least 2")
        return 1
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Switch the Percipio camera between modeling and fast modes "
        "(no robot motion).",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="print resolution, real fps and health")
    subparsers.add_parser("fast", help="switch to the first fast candidate with fps >= 10")
    subparsers.add_parser("modeling", help="switch back to 1280x960")
    subparsers.add_parser("probe", help="measure every candidate mode and print a table")
    args = parser.parse_args(argv)

    ensure_ros_environment()

    handlers = {
        "status": cmd_status,
        "fast": cmd_fast,
        "modeling": cmd_modeling,
        "probe": cmd_probe,
    }
    try:
        return handlers[args.command](args)
    except KeyboardInterrupt:
        log("\n[camera] interrupted; best-effort restore of modeling mode")
        try:
            restored = run_spec(MODELING_SPEC)
            if restored["color_fps"] > 0:
                print_measurement("modeling", restored)
        except Exception as exc:  # pragma: no cover - best effort only
            log(f"[camera] restore failed: {exc}")
        return 130


if __name__ == "__main__":
    sys.exit(main())
