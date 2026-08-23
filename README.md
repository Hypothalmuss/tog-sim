# tog-sim — vision-guided high-speed pick & place cell (ROS 2 Humble · Gazebo Fortress)

> Work in progress — an open, simulated re-interpretation of a packaging cobot *inspired by the Schubert tog.519*:
> a fast SCARA that **segments unsorted products with a neural network, checks tray pockets for vacancy,
> picks with a vacuum gripper and places into moving trays** — run from an industrial web HMI.

**Hardware modelled (all off-the-shelf):** Epson GX8-C653S SCARA · tog-sim tilt module (5th axis) · OnRobot VGC10 vacuum gripper ·
Intel RealSense D435 · Open-RMF workcell / conveyors / trays · Google Scanned Objects & YCB products.
See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

![tog-sim cell: Epson GX8 SCARA with tilt module and VGC10 suction cup between an infeed belt (products) and an outfeed belt (trays), two overhead D435 cameras, Open-RMF enclosure — Gazebo Fortress](docs/media/setup.png)

## Status

| Milestone | State |
|---|---|
| M0 Skeleton — packages, robot description, analytical IK + tests, CI | ✅ |
| M1 Sim cell — conveyors, products, trays, ros2_control in Gazebo Fortress | ✅ |
| M2 First closed loop (ground-truth perception) | ✅ 12/12 GT cycles, ~32 cpm motion-only |
| M3 Vision — synthetic data, YOLO-seg, grasp pose, tray vacancy | ✅ 1773 synthetic scenes (a third with bar trays) → YOLO11n-seg (mask mAP50 0.98); vs ground truth: class acc 1.00, grasp height 0.4 mm, yaw 0.8°; tray pose 4 mm / 0.05°, pocket occupancy validated (`eval_pick_poses`, `eval_tray_state`) |
| M4 Speed & moving-tray tracking, benchmark | ✅ `conveyor_tracker` (stable product/tray tracks → TF) + `TRACK_CART` picks/places on **moving belts, no belt stops**; 120/120 cartons at 13–14.5 picks/min, placement 2.7–3.7 mm mean with 95 % CIs; bars on the 620 mm tray 4.4 mm (see Benchmarks) |
| M5 HMI | ✅ `togsim_hmi`: FastAPI + vanilla JS operator panel - status, belts, recipes, start/stop/e-stop, alarms with acknowledge, health, run history, tray occupancy |
| M6 Polish, teach-in, stereo stretch, Docker | ⬜ |

## Demos

| Cartons only, 60 products/min, `fast` motion profile, vision perception — live from Gazebo |
|---|
| ![demo](docs/media/demo_speed.gif) |

Continuous vision-driven pick & place from the *moving* belts (YOLO11n-seg → pick poses → conveyor tracker →
`TRACK_CART` motions), recorded with `scripts/demo.sh vision 30` (`DEMO_CLASSES=product_carton DEMO_RATE=60.0
DEMO_PROFILE=fast DEMO_OUTFEED=0.06`): 7 cycles in the 40 s clip, the first four back-to-back at ~20 picks/min, then
waits for the next tray; products land within a few mm of the pocket centre (numbers in [Benchmarks](#benchmarks)).
The operator HMI (`http://localhost:8080`) runs alongside.

## Quick start (native)

```bash
# Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (ros-humble-ros-gz, ros-humble-ign-ros2-control)
git clone <this repo> ~/tog-sim && cd ~/tog-sim   # use a real directory (no symlinks, no spaces in the path)
./scripts/fetch_assets.sh          # third-party meshes + Fuel models
colcon build   # packages live in src/; do not use --symlink-install: Gazebo's model:// lookup does not follow symlinked model dirs
source scripts/env.sh   # in EVERY shell: own ROS_DOMAIN_ID (42), workspace overlay, GPU rendering, optional ~/togsim_data/venv
ros2 launch togsim_description view_robot.launch.py      # robot in RViz with joint sliders
ros2 launch togsim_gazebo sim.launch.py                   # full cell in Gazebo Fortress (gui:=false for headless)
ros2 launch togsim_bringup sim_full.launch.py gui:=false  # cell + controllers + vacuum + motion server (headless)
./scripts/killall.sh                                      # stop leftover tog-sim processes only (stale bridges publish /clock)
```

tog-sim uses its own `ROS_DOMAIN_ID` (default 42, override with `TOGSIM_ROS_DOMAIN_ID`) so that other ROS 2 stacks on the
same machine or LAN — in particular another Gazebo publishing `/clock` — can never mix with it.

### Vision (M3)

```bash
ros2 launch togsim_bringup sim_full.launch.py gui:=false products:=false segmentation:=true   # cell with panoptic cameras
ros2 run togsim_perception run_datagen --ros-args -p frames:=400      # resumable synthetic dataset -> ~/togsim_data/seg_v1
ros2 run togsim_perception train_seg -- --epochs 40                   # YOLO11n-seg -> ~/togsim_data/weights/togsim_seg.pt
ros2 launch togsim_bringup sim_full.launch.py gui:=false          # populated cell (segmentation:=true only needed for datagen)
ros2 launch togsim_perception perception.launch.py                    # segmentation + pick poses + tray vacancy
ros2 run togsim_perception eval_pick_poses --ros-args -p use_sim_time:=true   # vision vs ground truth metrics
ros2 run togsim_task run_cycle --ros-args -p perception:=vision -p use_sim_time:=true
```

Note: ultralytics treats numpy images as BGR - the nodes convert the ROS `rgb8` frames before inference (training PNGs are BGR).

GPU: install a `torch` build matching the NVIDIA driver into `~/togsim_data/venv` (`python3 -m venv --system-site-packages`,
e.g. `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128` for driver 535); `scripts/env.sh`
activates it when present and the nodes pick CUDA automatically (`device:=auto`).

## Benchmarks

`scripts/bench.sh <name> [repeats] [cycles] [vision|gt]` runs seeded repeats of `scripts/m4_validate.sh` and writes one JSON
per run (cycles, attempts, cpm, motion time, placement mean/p95 from ground truth, per-phase timeline, failures);
`scripts/bench_report.py <name> ...` aggregates them with 95 % confidence intervals into
[docs/benchmarks.md](docs/benchmarks.md) (Wilson for success, bootstrap over cycles for picks/min and over placed
products for placement). The table is regenerated; the reading of the numbers is kept under its `## Notes`. Placement = released product vs the centre of the pocket it sits in (ground truth); the pocket
clearance is 5 mm per side. The tuned perception gates live in
[`src/togsim_perception/config/perception_tuning.yaml`](src/togsim_perception/config/perception_tuning.yaml) and the
cycle parameters in [`src/togsim_task/config/task_tuning.yaml`](src/togsim_task/config/task_tuning.yaml), each with its
rationale and the levers rejected on the benches. `run_cycle` is one cycle in four phase methods - `schedule` (what is pickable now and where it can
go), `pick` (fly, track, seal; measures the grasp offset), `choose_pocket` (after the pick: the tray chosen before has
moved on) and `place` (track the pocket, release, unseal) - with the run constants in a `CycleConfig`; the scheduling
rules are in `togsim_task/scheduler.py` (unit-tested) and every ground-truth diagnostic in `togsim_task/diagnostics.py`
(`eval:=false` for a plain demo).

**Reference (cartons 60/min, `fast`, outfeed 0.06 m/s, 2×40 cycles with every fix below): 80/80 = 100 % [95–100],
13.6 [12.2–15.4] picks/min, placement 3.2 [2.8–3.7] mm mean, 6.2 [5.6–8.8] mm p95; the 3×40 baseline before the
bar-tray work was 13.4 [12.2–14.7] picks/min, 2.7 [2.4–3.0] / 5.8 [4.8–7.1] mm** — see the report for every
configuration and for the rejected levers.

| scenario (vision) | success | picks/min | placement mean | p95 |
|---|---|---|---|---|
| cartons only, 60/min, `fast`, outfeed 0.06 m/s (2×40) | 80/80 | 13.6 [12.2–15.4] | 3.2 [2.8–3.7] mm | 6.2 [5.6–8.8] mm |
| bars only on `tray_bar_2x3`, 30/min (12 cycles) | 12/12, no tray disturbed | 11.9 | 4.4 mm | 6.1 mm |
| cartons + bars 30/min on `tray_2x4` + `tray_bar_2x3` (2×20) | 40/42 = 95 % [84–99] | 7.2 [5.5–9.8] (supply-bound) | 5.1 [3.6–7.0] mm | 8.7 mm |

Motion profiles (`motion_profile:=fast|smooth`, `scripts/joint_metrics.py` samples `/joint_states`, cartons 60/min):

| profile | cpm | motion / cycle | acc p95 J1/J2/J4 (rad/s²) | jerk p95 J1/J2/J4 (rad/s³) |
|---|---|---|---|---|
| fast | 7–10 | 3.2–4.2 s | 40 / 45 / 51 | 457 / 491 / 1391 |
| smooth | 6–7 | 4.4–4.5 s | 22 / 26 / 33 | 213 / 223 / 833 |
| max (`profile_max.yaml`, bench only) | 13.7 (outfeed 0.06) | 2.4 s | 63 / 55 / 95 | 1627 / 1292 / 2592 |

What made the difference (details in the commit log): tracked segments settle on the *measured* arm including the
heading, transfers clear the pocket walls by 50 mm and the cup keeps tracking the pocket until the product has let
go, the tracker tracks the product *centre* (a suction point jumps along an elongated product between frames) and
the settle error measured at the seal is compensated at the place, tray tracks are dead-reckoned on a shared
belt-speed estimate (trays ride the 0.06 m/s outfeed at 0.050 m/s) and observations taken while the arm is over a
tray are ignored; the look-ahead and the belt-time waits scale with the measured real-time factor. Throughput is bounded by the tray window: a tray is placeable for
~3 s of its passage at 0.10 m/s, so cycles of ~3 s allow one or two placements per tray; a slower outfeed (0.06 m/s)
widens the window and also improves precision (the benches above run at 0.06 m/s).

Several tray models can share the outfeed (`tray_models:=tray_2x4,tray_bar_2x3` on `sim_full.launch.py` and
`perception.launch.py`): the vacancy node picks the spec per mask from the pocket size and `run_cycle` places each
product class only into a fitting tray. Bars on the 620 mm `tray_bar_2x3` place at 4.4 mm mean / 6.1 mm p95
(12/12, no tray disturbed, 30 bars/min); the tray is placeable while fully in the place camera's view. Getting there
took five fixes, each confirmed against ground truth (the tracker now tracks the product centre instead of a
suction point that jumps along a bar, transfers clear the pocket walls by 50 mm, the bar pocket is sized to the
measured spread, the tray-pose gates handle a 200 mm pitch, the spawner keeps the tray pitch above the tray
lengths) - the story is in the Notes of [docs/benchmarks.md](docs/benchmarks.md).

## Operator HMI (M5)

```bash
ros2 launch togsim_hmi hmi.launch.py port:=8080        # with the cell running; scripts/demo.sh starts it too
# http://localhost:8080 : state / cycles / rate / success / placement, belt speed sliders, start-stop of the
# continuous pick & place (vision or ground truth), tray occupancy grid, pickable products, events, run_cycle log
```

The panel talks to a small rclpy bridge (`/togsim/hmi/status` from `run_cycle`, tracked trays/products, vacuum, joints,
belt `cmd_vel`) through `GET /api/state`, `POST /api/belts`, `POST /api/run`, `POST /api/stop`.

## Packages

| Package | Role |
|---|---|
| `src/togsim_msgs` | Interfaces (`PickCandidate`, `TrayState`, `VacuumState`, `CycleEvent`, `Kpi`, `ExecuteMotion` action …) |
| `src/togsim_description` | Epson GX8-C653S + tilt module + VGC10 xacros, limits, RViz launch |
| `src/togsim_gazebo` | Fortress world, product/tray models, **custom vacuum gripper system plugin**, spawner |
| `src/togsim_control` | ros2_control controllers, vacuum bridge |
| `src/togsim_motion` | Analytical SCARA IK, Ruckig 500 Hz streaming, `ExecuteMotion` server, conveyor tracker |
| `src/togsim_perception` | Synthetic dataset generation, YOLO-seg training/inference, grasp pose, tray vacancy, teach-in |
| `src/togsim_task` | Lifecycle FSM + py_trees pick/place cycle, KPIs, benchmark |
| `src/togsim_hmi` | FastAPI + vanilla JS operator HMI |
| `src/togsim_bringup` | Top-level launch files, Docker |

## Licence

Apache-2.0 (see `LICENSE`). Third-party assets keep their own licences (`THIRD_PARTY_NOTICES.md`).
