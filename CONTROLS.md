# Quick Reference — Stretch MuJoCo Digital Twin

## Setup (first time only)

**1. Install `uv` (package manager)**

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Restart your terminal after installing, then verify:

```bash
uv --version
```

**2. Clone the repo**

```bash
git clone https://github.com/luisodmg/Stretch-Robot-Repository --recurse-submodules
cd Stretch-Robot-Repository
```

Already cloned but missing submodules? Run:

```bash
git submodule update --init
```

**3. Install dependencies**

```bash
uv sync
```

That's it — `uv` handles the Python version (3.10) and all packages automatically.

> **Linux only:** if you get a build error mentioning `evdev` or `gcc`, install:
> ```bash
> sudo apt install build-essential python3-dev linux-headers-$(uname -r)
> ```
> Then run `uv sync` again.

---

## Running the simulation

```bash
uv run teleop_demo.py
```

The first run takes longer while the environment loads. You should see in the terminal:

```
=== Running on simulation backend ===
```

And a MuJoCo viewer window with the Stretch robot.

> **Important:** before controlling the robot, click **outside** the MuJoCo window
> (e.g. on the taskbar). If the viewer has focus, keys move the camera instead of the robot.

---

## Keyboard controls

### Base movement

| Key | Action |
|---|---|
| `W` | Move forward |
| `S` | Move backward |
| `D` | Turn right |
| `A` | Turn left |

### Arm

| Key | Action |
|---|---|
| `Z` | Lift up |
| `X` | Lift down |
| `V` | Extend arm (out) |
| `C` | Retract arm (in) |

### Gripper & wrist

| Key | Action |
|---|---|
| `M` | Open gripper |
| `N` | Close gripper |
| `L` | Wrist yaw left |
| `J` | Wrist yaw right |
| `U` | Wrist roll left |
| `O` | Wrist roll right |
| `K` | Wrist pitch up |
| `I` | Wrist pitch down |

### Head

> Activate head mode first with `H`.

| Key | Action |
|---|---|
| `H` | Toggle **head** / **wrist** mode |
| `L` / `J` | Head pan (in head mode) |
| `K` / `I` | Head tilt (in head mode) |

### Camera views

Each key toggles that camera window on/off. Having multiple active will drop FPS.

| Key | Camera |
|---|---|
| `1` | Head RGB |
| `2` | Head Depth |
| `3` | Wrist RGB |
| `4` | Wrist Depth |
| `5` | Navigation Cam |

### Other

| Key | Action |
|---|---|
| `Y` | Toggle manual mode |
| `Ctrl+C` | Quit |

---

## Gamepad (Xbox)

If a gamepad is connected, it takes priority over keyboard input.

| Input | Action |
|---|---|
| `LY` (left stick vertical) | Base forward / backward |
| `LX` (left stick horizontal) | Turn base |
| `RY` (right stick vertical) | Lift up / down |
| `RX` (right stick horizontal) | Extend / retract arm |
| `A` / `B` | Close / open gripper |
| `LB` / `RB` | Wrist yaw |
| `DPAD_X` | Wrist roll |
| `DPAD_Y` | Wrist pitch / head tilt |
| `X` | Toggle head / wrist mode |
| `Y` | Toggle manual mode |

---

## Environment: simple blocks vs RoboCasa kitchen

Configured in `stretch_toolkit/robocasa_config.json`:

```json
{
  "enabled": false,   ← true for kitchen, false for simple blocks
  "task": "PnPCounterToCab",
  "layout": 0,
  "style": 0
}
```

| `enabled` | Environment |
|---|---|
| `false` | Simple room with a blue box and red cylinder |
| `true` | Full kitchen with cabinets, appliances and objects |

Available RoboCasa tasks: `PnPCounterToCab`, `PnPCounterToSink`, `PnPSinkToCounter`, and others.

---

## Common issues

**Simulation won't start**
```bash
uv sync
uv run teleop_demo.py
```

**Controls not responding**
Click outside the MuJoCo viewer window before pressing keys.

**Low FPS / slow**
Disable camera feeds you're not using (keys `1`–`5`). The RoboCasa kitchen environment is heavier than the default one.

**Robot behaving oddly / physics glitch**
`Ctrl+C` and restart the simulation.

**Linux: build error mentioning `evdev` or `gcc`**
```bash
sudo apt install build-essential python3-dev linux-headers-$(uname -r)
```
