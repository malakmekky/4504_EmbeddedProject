#!/usr/bin/env python3
"""
PC controller for the maze robot (ECNG 4504, phase 1).

Connects to the robot over BLE, sends commands, shows live progress,
and logs every message to pc_app/logs/.

    python maze_controller.py              # find "LineBot" and connect
    python maze_controller.py --name Foo   # different device name

Commands at the prompt:
    train   map the whole maze, robot sends back the map + shortest path
    solve   run the shortest path
    stop    stop the motors now
    path    ask the robot for the stored path
    status  ask the robot what it is doing
    forget  erase the saved path + map (new maze, or starting from the other circle)
    quit    disconnect and exit
"""

import argparse
import asyncio
import datetime
import json
import pathlib
import subprocess
import sys

from bleak import BleakClient, BleakScanner

CMD_UUID = "7a1e0002-3c2b-4d8e-9f10-2b6c5d4e3f21"
STATUS_UUID = "7a1e0003-3c2b-4d8e-9f10-2b6c5d4e3f21"

HERE = pathlib.Path(__file__).resolve().parent
LOG_DIR = HERE / "logs"
PATH_FILE = HERE / "last_path.json"

MOVE_NAMES = {"L": "LEFT", "S": "STRAIGHT", "R": "RIGHT", "B": "BACK"}
ARROWS = {"L": "←", "S": "↑", "R": "→", "B": "↓"}

COMMANDS = {
    "t": "TRAIN", "train": "TRAIN",
    "s": "SOLVE", "solve": "SOLVE",
    "x": "STOP", "stop": "STOP",
    "p": "PATH", "path": "PATH",
    "?": "STATUS", "status": "STATUS",
    "f": "FORGET", "forget": "FORGET",
}


def now():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def describe(path):
    """'RSL' -> 'RIGHT → STRAIGHT → LEFT'"""
    return "  ".join(f"{ARROWS.get(m, m)} {MOVE_NAMES.get(m, m)}" for m in path) or "(empty)"


def draw_map(nodes, edges, path_ids):
    """Draw the maze map the robot built. Returns the PNG path, or None without matplotlib."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    fig, ax = plt.subplots(figsize=(7, 7))
    on_path = set(zip(path_ids, path_ids[1:])) | set(zip(path_ids[1:], path_ids))

    for a, b, length in edges:
        (xa, ya), (xb, yb) = nodes[a]["pos"], nodes[b]["pos"]
        best = (a, b) in on_path
        ax.plot([xa, xb], [ya, yb], color="#2563eb" if best else "#9ca3af",
                linewidth=5 if best else 2, zorder=1)
        ax.annotate(f"{length / 1000:.1f}s", ((xa + xb) / 2, (ya + yb) / 2),
                    fontsize=7, color="#6b7280", ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))

    for i, n in nodes.items():
        x, y = n["pos"]
        color = {"S": "#16a34a", "E": "#dc2626"}.get(n["tag"], "#111827")
        ax.scatter([x], [y], s=160 if n["tag"] != "-" else 60, color=color, zorder=2)
        label = {"S": "START", "E": "END"}.get(n["tag"], str(i))
        ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 6), fontsize=9)

    ax.set_aspect("equal")
    ax.set_title("Maze map from training (blue = shortest path)")
    ax.set_xlabel("x (ms of driving)")
    ax.set_ylabel("y (ms of driving)")
    ax.grid(alpha=0.3)

    out = LOG_DIR / f"map_{datetime.datetime.now():%Y%m%d_%H%M%S}.png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return out


class RobotLink:
    """Turns BLE notifications into lines and reacts to them."""

    def __init__(self, log_file):
        self.buffer = ""
        self.log = log_file
        self.raw_path = ""
        self.best_path = ""
        self.map_nodes = {}
        self.map_edges = []
        self.map_path = []

    def on_notify(self, _char, data: bytearray):
        # long lines arrive in pieces, a full line ends with '\n'
        self.buffer += data.decode(errors="replace")
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            line = line.strip()
            if line:
                self.handle(line)

    def handle(self, line):
        self.log.write(f"{now()}  {line}\n")
        self.log.flush()

        word, _, rest = line.partition(" ")

        if word == "STEP":
            # "STEP 3/7 R" -> progress bar
            frac, _, move = rest.partition(" ")
            done, total = (int(x) for x in frac.split("/"))
            bar = "#" * done + "-" * (total - done)
            print(f"\r[{now()}] SOLVING [{bar}] {done}/{total}  {ARROWS.get(move, '')} {MOVE_NAMES.get(move, move)}")

        elif word == "NODE":
            print(f"[{now()}] TRAINING  node {rest}")

        elif word == "MAP":
            self.handle_map(rest)

        elif word == "PATH":
            kind, _, path = rest.partition(" ")
            if kind == "RAW":
                self.raw_path = path
                print(f"[{now()}] explored path : {path or '(empty)'}")
            elif kind == "BEST":
                self.best_path = path
                print(f"[{now()}] shortest path : {path or '(empty)'}")
                print(f"           {describe(path)}")
                self.save_path()

        elif word in ("TRAIN_DONE", "SOLVE_DONE"):
            print(f"\n[{now()}] *** {word.replace('_', ' ')}  {rest} ***\n")

        elif word in ("ERR", "WARN"):
            print(f"[{now()}] !! {line}")

        else:
            print(f"[{now()}] {line}")

    def handle_map(self, rest):
        kind, *f = rest.split()
        if kind == "BEGIN":
            self.map_nodes, self.map_edges, self.map_path = {}, [], []
        elif kind == "N":       # N id x y exits tag
            self.map_nodes[int(f[0])] = {"pos": (int(f[1]), int(f[2])), "exits": f[3], "tag": f[4]}
        elif kind == "E":       # E a b length
            self.map_edges.append((int(f[0]), int(f[1]), int(f[2])))
        elif kind == "P":       # P id id id ...
            self.map_path += [int(x) for x in f]
        elif kind == "END":
            print(f"[{now()}] map: {len(self.map_nodes)} nodes, {len(self.map_edges)} edges, "
                  f"shortest path through nodes {self.map_path}")
            self.save_path()
            png = draw_map(self.map_nodes, self.map_edges, self.map_path)
            if png is None:
                print("           (pip install matplotlib to get a picture of the map)")
            else:
                print(f"           map saved to {png}")
                if sys.platform == "darwin":
                    subprocess.Popen(["open", str(png)])

    def save_path(self):
        PATH_FILE.write_text(json.dumps({
            "time": datetime.datetime.now().isoformat(timespec="seconds"),
            "explored": self.raw_path,
            "shortest": self.best_path,
            "map": {
                "nodes": {str(k): v for k, v in self.map_nodes.items()},
                "edges": self.map_edges,
                "shortest_nodes": self.map_path,
            },
        }, indent=2))


async def ainput(prompt=""):
    return await asyncio.get_running_loop().run_in_executor(None, input, prompt)


# Only one keyboard read is ever waiting, shared across reconnects. (Before, a read left
# over from a lost connection swallowed the first command typed after reconnecting.)
_pending_input = None


def next_line_task():
    global _pending_input
    if _pending_input is None:
        _pending_input = asyncio.ensure_future(ainput())
    return _pending_input


def take_line():
    global _pending_input
    task, _pending_input = _pending_input, None
    return task.result()


async def read_line(prompt=""):
    if prompt:
        print(prompt, end="", flush=True)
    await next_line_task()
    return take_line()


async def find_robot(name):
    print(f"Scanning for '{name}' ...")
    device = await BleakScanner.find_device_by_name(name, timeout=10.0)
    if device is None:
        print("Robot not found. Is it powered on and not connected to another device?")
    return device


async def session(device, link):
    disconnected = asyncio.Event()

    async with BleakClient(device, disconnected_callback=lambda _c: disconnected.set()) as client:
        print(f"Connected to {device.name} ({device.address})")
        await client.start_notify(STATUS_UUID, link.on_notify)
        print("Commands: train | solve | stop | path | status | forget | quit\n")

        while not disconnected.is_set():
            prompt_task = next_line_task()
            lost_task = asyncio.create_task(disconnected.wait())
            done, _ = await asyncio.wait({prompt_task, lost_task}, return_when=asyncio.FIRST_COMPLETED)

            if lost_task in done:
                print("\nConnection lost.")
                return True          # reconnect (a half-typed line is kept for the next session)

            lost_task.cancel()
            text = take_line().strip().lower()

            if text in ("q", "quit", "exit"):
                await client.write_gatt_char(CMD_UUID, b"STOP", response=True)
                return False

            if not text:
                continue

            cmd = COMMANDS.get(text)
            if cmd is None:
                print("Unknown command. Use: train | solve | stop | path | status | forget | quit")
                continue

            await client.write_gatt_char(CMD_UUID, cmd.encode(), response=True)
            link.log.write(f"{now()}  >> {cmd}\n")

    return True


async def main():
    parser = argparse.ArgumentParser(description="BLE controller for the maze robot")
    parser.add_argument("--name", default="LineBot", help="BLE name of the robot")
    args = parser.parse_args()

    LOG_DIR.mkdir(exist_ok=True)
    log_path = LOG_DIR / f"run_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"

    with open(log_path, "w") as log_file:
        print(f"Logging to {log_path}")
        link = RobotLink(log_file)

        while True:
            device = await find_robot(args.name)
            if device is None:
                answer = await read_line("Try again? [Y/n] ")
                if answer.strip().lower() == "n":
                    return
                continue

            try:
                if not await session(device, link):
                    return
            except Exception as e:  # noqa: BLE001 - keep the app alive and reconnect
                print(f"BLE error: {e}")

            print("Reconnecting ...")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
