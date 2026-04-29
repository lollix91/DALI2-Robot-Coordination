"""
Single-command launcher for the DALI2 Multi-Robot Search-and-Rescue demo.

Starts all four components in order:
  1. Redis (via Docker, if not already running)
  2. CoppeliaSim scene builder
  3. DALI2 agent server
  4. Python bridge

Usage:
    python launch.py [--skip-redis] [--skip-scene] [--config scene/scenario.yaml]
    python launch.py --openrouter sk-or-v1-abc123...   # use OpenRouter gpt-4o

Press Ctrl+C to stop everything.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
DALI2_DIR = os.path.normpath(os.path.join(ROOT, "..", "DALI2"))
SERVER_PL = os.path.join(DALI2_DIR, "src", "server.pl")


def is_redis_running() -> bool:
    """Check if dali2-redis container is running."""
    try:
        out = subprocess.check_output(
            ["docker", "ps", "--filter", "name=dali2-redis",
             "--format", "{{.Names}}"],
            stderr=subprocess.DEVNULL, text=True
        )
        return "dali2-redis" in out
    except Exception:
        return False


def start_redis() -> None:
    """Start the Redis container if not already running."""
    if is_redis_running():
        print("[launch] Redis already running (dali2-redis)")
        return
    # Try to start existing stopped container first
    try:
        subprocess.check_call(
            ["docker", "start", "dali2-redis"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        print("[launch] Redis container restarted (dali2-redis)")
        return
    except Exception:
        pass
    # Create new container
    print("[launch] Starting Redis container...")
    subprocess.check_call(
        ["docker", "run", "-d", "--name", "dali2-redis",
         "-p", "6379:6379", "redis:7-alpine"],
        stdout=subprocess.DEVNULL
    )
    print("[launch] Redis started on port 6379")
    time.sleep(1)  # Give Redis a moment to be ready


def build_scene(config: str | None, pioneer_model: str | None) -> None:
    """Build the CoppeliaSim scene."""
    print("[launch] Building CoppeliaSim scene...")
    cmd = [sys.executable, os.path.join(ROOT, "scene", "build_scene.py")]
    if config:
        cmd += ["--config", config]
    if pioneer_model:
        cmd += ["--pioneer-model", pioneer_model]
    subprocess.check_call(cmd, cwd=ROOT)


def main() -> None:
    p = argparse.ArgumentParser(description="Launch the full DALI2 rescue demo")
    p.add_argument("--skip-redis", action="store_true",
                   help="Skip Redis startup (assume already running)")
    p.add_argument("--skip-scene", action="store_true",
                   help="Skip scene building (scene already populated)")
    p.add_argument("--config", default=None,
                   help="YAML scenario config for scene + bridge")
    p.add_argument("--pioneer-model", default=None,
                   help="Absolute path to Pioneer P3DX .ttm model")
    p.add_argument("--port", type=int, default=8080,
                   help="DALI2 HTTP server port (default: 8080)")
    p.add_argument("--agent-file", default="agents/rescue.pl",
                   help="Path to DALI2 agent file (default: agents/rescue.pl)")
    p.add_argument("--openrouter", metavar="API_KEY", default=None,
                   help="Use OpenRouter with gpt-4o for vision instead of "
                        "local Jan. Pass your OpenRouter API key.")
    args = p.parse_args()

    procs: list[subprocess.Popen] = []

    def cleanup(sig=None, frame=None):
        print("\n[launch] Shutting down...")
        for proc in reversed(procs):
            try:
                if sys.platform == "win32":
                    # On Windows, terminate the entire process tree
                    subprocess.call(
                        ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                else:
                    proc.terminate()
                proc.wait(timeout=5)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
        print("[launch] All processes stopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        # 1. Redis
        if not args.skip_redis:
            start_redis()

        # 2. Build scene
        if not args.skip_scene:
            build_scene(args.config, args.pioneer_model)

        # 3. Bridge (start FIRST so it subscribes to LINDA before agents)
        print("[launch] Starting CoppeliaSim bridge...")
        bridge_cmd = [sys.executable, os.path.join(ROOT, "bridge", "coppelia_bridge.py")]
        if args.config:
            bridge_cmd += ["--config", args.config]

        # Build environment for the bridge process
        bridge_env = os.environ.copy()
        if args.openrouter:
            bridge_env["VISION_LLM_ENDPOINT"] = (
                "https://openrouter.ai/api/v1/chat/completions"
            )
            bridge_env["VISION_LLM_MODEL"] = "openai/gpt-4o"
            bridge_env["VISION_LLM_API_KEY"] = args.openrouter
            bridge_env["VISION_LLM_TIMEOUT"] = "30"  # remote API, allow more time
            print("[launch] VLM backend: OpenRouter (gpt-4o)")
        else:
            print("[launch] VLM backend: Jan local (Qwen3)")

        bridge_proc = subprocess.Popen(bridge_cmd, cwd=ROOT, env=bridge_env)
        procs.append(bridge_proc)
        time.sleep(3)  # Give bridge time to connect and subscribe to LINDA

        # 4. DALI2 server (agents will now find the bridge listening)
        print(f"[launch] Starting DALI2 server on port {args.port}...")
        dali2_cmd = [
            "swipl", "-l", SERVER_PL, "-g", "main",
            "--", str(args.port), args.agent_file
        ]
        dali2_proc = subprocess.Popen(dali2_cmd, cwd=ROOT)
        procs.append(dali2_proc)

        print("[launch] All components started. Press Ctrl+C to stop.")
        print(f"[launch] Web UI: http://localhost:{args.port}")

        # Wait for any process to exit
        while True:
            for proc in procs:
                ret = proc.poll()
                if ret is not None:
                    print(f"[launch] Process {proc.args[0]} exited with code {ret}")
                    cleanup()
            time.sleep(1)

    except Exception as e:
        print(f"[launch] Error: {e}")
        cleanup()


if __name__ == "__main__":
    main()
