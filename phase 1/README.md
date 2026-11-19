# ECNG 4504 Maze Robot – Phase 1

ESP32 (ESP-IDF v5.4 + FreeRTOS) line-maze robot that explores a maze, computes the
shortest path, and replays it, controlled from a Python app over BLE.

Team: Malak Mekky, Malak Taha, Jannah Hella

## Repository layout

| Path | What |
|---|---|
| `main/main.c` | Startup: NVS, BLE, starts the robot task |
| `main/robot.c` | **Finite state machine** (training + solving) |
| `main/maze.c` | Maze **graph**: coordinates, exploration, Dijkstra shortest path, flash storage |
| `main/ble_link.c` | NimBLE GATT server, command / status queues |
| `main/motors.c` | L298N driver (LEDC PWM) |
| `main/sensors.c` | 5-channel line sensor |
| `main/config.h` | All pins and tuning values |
| `pc_app/maze_controller.py` | Python BLE app for the PC |
| `legacy/phase0_esp32_port.c` | Phase 0 code ported to ESP32 (reference) |

## Wiring (ESP32 DevKit)

| Part | Pin | ESP32 |
|---|---|---|
| L298N | ENA / IN1 / IN2 (left motors) | GPIO21 / 19 / 18 |
| L298N | IN3 / IN4 / ENB (right motors) | GPIO22 / 23 / 4 |
| L298N | 12V in | Battery + (3S) |
| L298N | 5V out | ESP32 5V/VIN |
| HW-871 | OUT1 (leftmost) … OUT5 (rightmost) | GPIO25, 33, 32, 35, 34 |
| HW-871 | VCC | **ESP32 3V3** (GPIOs are not 5 V tolerant) |
| all | GND | common ground |

## Software architecture

| Task | Core | Priority | Job |
|---|---|---|---|
| `robot` | 1 | 5 | FSM, sensors, motors (runs alone on its core so BLE never delays it) |
| `ble_tx` | 0 | 4 | Sends status lines from `txQueue` as BLE notifications |
| NimBLE host | 0 | – | BLE stack; incoming commands go into `cmdQueue` |

### State machine (`robot.c`)

```
IDLE ──TRAIN/SOLVE──> LEAVE_START ──> FOLLOW
FOLLOW ──branch──> NODE ──L/R──> TURN ──> FOLLOW
                     ├──S─────────────> FOLLOW
                     ├──B──> UTURN ───> FOLLOW
                     ├──circle (training)──> EXIT_CIRCLE ──> UTURN
                     └──all explored / end reached──> FINISHED ──> IDLE
FOLLOW ──line ended──> UTURN (dead end)
FOLLOW ──drifted off──> RECOVER ──> FOLLOW
TURN/UTURN failed ──> RECOVER
any ──STOP──> IDLE
```

### Algorithm (`maze.c`)

**Map = graph.** Every place the car stops to decide (intersection, corner, dead end,
start/end circle) is a node that stores:

* **coordinates** `(x, y)`: the car has no wheel encoders, so distance = time spent
  driving along the line at `NORMAL_SPEED`. Each edge only changes one coordinate
  because all turns are 90°.
* **exits**: one boolean per direction N/E/S/W.
* **neighbours**: which node each exit leads to (or "not explored yet") and the edge length.

**Normalized direction.** The car leaves the start facing N. Every turn rotates the
heading by 90° (R = +1, L = −1, B = +2 steps), so what the sensors see (left /
straight / right) is converted into absolute directions.

**Recognising a node seen before (closing loops).**
* Driving down an edge that's already in the map → we know exactly which node we reach.
* Driving down a new edge → look for an existing node close to the estimated
  position, *ahead* of the car, with an unexplored exit pointing back at us. If one
  exists the loop is closed, otherwise a new node is created. The allowed error grows
  with the distance driven since the last node we were sure about (`MATCH_TOL_FRAC`).

**Systematic exploration (depth-first search).**
1. If the current node has an unexplored exit, take it (preference right, straight, left).
2. Otherwise run Dijkstra on the known graph and drive the shortest route to the
   nearest node that still has an unexplored exit (backtracking).
3. **Done criterion:** no node in the map has an unexplored exit.

**Shortest path.** Dijkstra from the start node to the end node over the edge lengths.
The node list is turned back into relative moves (`L S R`) for the solve run, which
replays them one intersection at a time.

**Tested in simulation** (`maze.c` compiled on a PC against simulated mazes with
loops and dead ends): correct shortest path in 300/300 runs with ±10% distance error
on every edge, 293/300 with ±15%.

**Line following:** PID on the sensor position (`KP`, `KI`, `KD` in `config.h`).

## BLE protocol

Device name `LineBot`, service `7a1e0001-3c2b-4d8e-9f10-2b6c5d4e3f21`

| Characteristic | UUID | Use |
|---|---|---|
| CMD | `7a1e0002-…` | write `TRAIN`, `SOLVE`, `STOP`, `PATH`, `STATUS` |
| STATUS | `7a1e0003-…` | notify, text lines ending in `\n` |

Messages from the robot:

| Message | Meaning |
|---|---|
| `STATE <name>` | FSM changed state |
| `NODE <id> (<x>,<y>) exits=<NESW> <new/known/loop/END> -> <move> <why>` | training decision |
| `PLAN go to node <id> (<n> edges)` | backtracking to the nearest unexplored branch |
| `SHORTEST length=<ms> through <n> nodes` | Dijkstra result |
| `PATH RAW <moves>` / `PATH BEST <moves>` | moves taken while exploring / shortest path |
| `MAP BEGIN`, `MAP N id x y exits tag`, `MAP E a b len`, `MAP P ids…`, `MAP END` | full map |
| `STEP <i>/<n> <move>` | solve progress |
| `TRAIN_DONE …`, `SOLVE_DONE …`, `WARN …`, `ERR …` | results and problems |

## Build and flash

```bash
. ~/esp/v5.4/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash        # add "monitor" to see the serial log
```

## PC app

```bash
cd pc_app
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
.venv/bin/python maze_controller.py
```

Type `train` and wait for `TRAIN DONE`, then place the robot back on the start and type `solve`.
Every session is logged in `pc_app/logs/`. After training, the app draws the map
(blue = shortest path) and saves it as a PNG in `pc_app/logs/`, and the map + path go
to `pc_app/last_path.json`.
