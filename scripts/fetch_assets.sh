#!/usr/bin/env bash
# Re-fetch all third-party assets used by tog-sim.
#  - Epson GX8-C653S meshes + property file (Apache-2.0)  -> togsim_description/meshes/epson_gx8_c653s
#  - OnRobot VGC10 1-cup meshes (MIT)                     -> togsim_description/meshes/onrobot_vgc10
#  - Gazebo Fuel models (CC BY 4.0)                       -> ~/.ignition/fuel cache (Gazebo resolves fuel:// URIs)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT

echo "[fetch] Epson GX8-C653S description"
git clone -q --depth 1 --filter=blob:none --sparse https://github.com/Epson-Robots/epson-robot-ros2.git "$TMP/epson"
(cd "$TMP/epson" && git sparse-checkout set epson_robot_description/models/GX8-C653S LICENSE >/dev/null)
mkdir -p "$ROOT/src/togsim_description/meshes/epson_gx8_c653s" "$ROOT/third_party/licenses"
cp "$TMP"/epson/epson_robot_description/models/GX8-C653S/meshes/*.stl "$ROOT/src/togsim_description/meshes/epson_gx8_c653s/"
cp "$TMP"/epson/epson_robot_description/models/GX8-C653S/urdf/epson_robot_property.xacro "$ROOT/src/togsim_description/urdf/epson_gx8_c653s_property.orig.xacro"
cp "$TMP/epson/LICENSE" "$ROOT/third_party/licenses/LICENSE.epson-robot-ros2"

echo "[fetch] OnRobot VGC10 description"
git clone -q --depth 1 --filter=blob:none --sparse https://github.com/UOsaka-Harada-Laboratory/onrobot.git "$TMP/onrobot"
(cd "$TMP/onrobot" && git sparse-checkout set onrobot_vg_description/meshes/vgc10 LICENSE >/dev/null)
mkdir -p "$ROOT/src/togsim_description/meshes/onrobot_vgc10/visual" "$ROOT/src/togsim_description/meshes/onrobot_vgc10/collision"
cp "$TMP"/onrobot/onrobot_vg_description/meshes/vgc10/visual/*1cup.stl "$ROOT/src/togsim_description/meshes/onrobot_vgc10/visual/"
cp "$TMP"/onrobot/onrobot_vg_description/meshes/vgc10/collision/*1cup.stl "$ROOT/src/togsim_description/meshes/onrobot_vgc10/collision/"
cp "$TMP/onrobot/LICENSE" "$ROOT/third_party/licenses/LICENSE.onrobot"

echo "[fetch] Gazebo Fuel models (cached under ~/.ignition/fuel)"
FUEL=(
  "Open-RMF/models/enclosure"
  "Open-RMF/models/conveyor_block"
  "Open-RMF/models/tray"
  "GoogleResearch/models/Nestle_Candy_19_oz_Butterfinger_Singles_116567"
  "Gambit/models/Cracker Box"
)
for m in "${FUEL[@]}"; do
  ign fuel download -u "https://fuel.gazebosim.org/1.0/$m" >/dev/null && echo "  ok  $m" || echo "  FAILED $m"
done
echo "[fetch] done"
