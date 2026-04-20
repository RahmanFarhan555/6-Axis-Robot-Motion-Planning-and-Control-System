# 6‑Axis Robot Control (Dockerized)

This repository implements a 6‑axis robot arm control demo with:

* A Python backend that stores joint state, enforces joint limits, generates smooth motions, and performs collision‑checked planning.
* A React/TypeScript frontend that visualizes a UR5 URDF and provides a simple control pad.
* Docker Compose for laptop-friendly, one-command startup.



## Requirements

* Docker + Docker Compose

## Quick start

From the repo root:

```bash
docker compose up --build
```

Open:

* Frontend: http://localhost:8080
* Backend API: http://localhost:8000

## What you can do in the UI

### Joint control

* Set joint targets (degrees) with sliders.
* Click **Move to targets** to execute.
* Jog individual joints using the ± buttons.
* Use **Home pose** to return to the default home posture.

### Pick \& Place demo

The project includes a fixed pick‑and‑place task:

* A cube starts on the pickup table.
* The robot picks it, avoids obstacles, places it on the place table, then returns to Home.

### Center bar obstacle height

The UI includes a **Center bar height** control.

* Range: "0.10" m … "0.90" m
* This updates the backend obstacle height and the 3D view.

## Coordinate system and scene

* Joint units: degrees
* Cartesian units: meters
* The robot is simulated in PyBullet (DIRECT mode).
* Tables and the bar are axis-aligned box obstacles.

Scene parameters (table positions, table height, bar size/position) are defined in [backend/app/scene.py](backend/app/scene.py) and mirrored visually in [frontend/src/components/RobotView.tsx](frontend/src/components/RobotView.tsx).

Pick \& place planning logic is implemented in [backend/app/pick\_place.py](backend/app/pick_place.py) and called by the API in [backend/app/main.py](backend/app/main.py).

## Backend API

Base URL: `http://localhost:8000`

### State

```bash
curl -s http://localhost:8000/api/state
```

### Set joint target (smooth motion)

```bash
curl -s -X POST http://localhost:8000/api/target \\
  -H 'Content-Type: application/json' \\
  -d '{"joints\_deg":\[90,-90,90,-90,0,0],"move\_seconds":2.0}'
```

### Jog a joint

```bash
curl -s -X POST http://localhost:8000/api/jog \\
  -H 'Content-Type: application/json' \\
  -d '{"joint\_index":1,"delta\_deg":5,"move\_seconds":0.5}'
```

### Run the fixed pick \& place task

```bash
curl -s -X POST http://localhost:8000/api/run\_pick\_place \\
  -H 'Content-Type: application/json' \\
  -d '{"move\_seconds":1.2}'
```

### Bar height

```bash
curl -s http://localhost:8000/api/bar\_height
curl -s -X POST http://localhost:8000/api/bar\_height \\
  -H 'Content-Type: application/json' \\
  -d '{"height":0.6}'
```

### WebSocket state stream

* Endpoint: `ws://localhost:8000/ws`
* The backend periodically broadcasts the current `RobotState`.

## Path planning and obstacle avoidance

Planning runs in the backend and is collision-checked against:

* The fixed obstacles (tables + center bar)
* Robot self-collision rules (non-adjacent contact disallowed, plus an end-effector clearance check)

Planner behavior:

* First tries a smooth joint-space interpolation for a segment.
* If the direct segment is invalid, falls back to **RRT‑Connect** in joint space.
* Performs waypoint shortcutting to reduce unnecessary detours.

Collision checking is performed via PyBullet contact/proximity queries with a small safety margin.

## Running tests

Run tests inside the backend container:

```bash
docker compose run --rm backend pytest -q /app/tests
```

## Development notes

* Images are built into Docker containers (no source bind mounts by default). After code changes, rebuild:

```bash
docker compose up -d --build
```

* Frontend can be pointed at a different backend by setting `VITE\_API\_BASE` at build time.

## Decisions and assumptions (for submission)

* **FastAPI + WebSocket**: simple JSON control APIs plus push-based state updates.
* **UR5 URDF visualization**: realistic 6-axis kinematics with clear joint motion.
* **Cubic interpolation + RRT‑Connect fallback**: fast “direct path” when possible, sampling-based planning when blocked.
* **Obstacle avoidance**: done by collision-checked planning (not reactive control).

