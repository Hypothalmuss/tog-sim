# Third-party assets and attributions

tog-sim is an independent, non-commercial, educational re-interpretation *inspired by* the Schubert tog.519 packaging cobot.
It is not affiliated with or endorsed by Gerhard Schubert GmbH, Seiko Epson, OnRobot, Intel, or Open Robotics.

| Asset | Source | Licence | Used for |
|---|---|---|---|
| Epson GX8-C653S SCARA meshes (`src/togsim_description/meshes/epson_gx8_c653s/*.stl`) and joint limits (`epson_gx8_c653s_property.orig.xacro`) | https://github.com/Epson-Robots/epson-robot-ros2 (`epson_robot_description`) | Apache-2.0 (`third_party/licenses/LICENSE.epson-robot-ros2`) | Robot visual/collision geometry and datasheet joint limits |
| OnRobot VGC10 (1 cup) meshes (`src/togsim_description/meshes/onrobot_vgc10/**`) | https://github.com/UOsaka-Harada-Laboratory/onrobot (`onrobot_vg_description`) | MIT (`third_party/licenses/LICENSE.onrobot`) | Vacuum gripper visual/collision geometry |
| Intel RealSense D435 description | `ros-humble-realsense2-description` | Apache-2.0 | Camera visual model and optical frames |
| Open-RMF `enclosure`, `conveyor_block`, `tray` | https://app.gazebosim.org/Open-RMF (Gazebo Fuel) | CC BY 4.0 | Workcell, conveyor bodies, tray carrier (downloaded at runtime by Gazebo) |
| Google Scanned Objects (e.g. `Nestle_Candy_19_oz_Butterfinger_Singles_116567`) | https://app.gazebosim.org/GoogleResearch (Gazebo Fuel) | CC BY 4.0 | Photoreal products |
| YCB object set (cracker box, gelatin box, pudding box) | https://www.ycbbenchmarks.com / Gazebo Fuel (`Gambit/Cracker Box`) | see YCB terms (research use), CC BY 4.0 on Fuel | Small carton products |
| Ruckig (community edition) | https://github.com/pantor/ruckig | MIT | Online trajectory generation |
| Ultralytics YOLO11 | https://github.com/ultralytics/ultralytics | AGPL-3.0 (weights/code used unmodified as a dependency; training and inference scripts in this repo are Apache-2.0) | Instance segmentation |
| uPlot | https://github.com/leeoniya/uPlot | MIT | HMI charts (vendored in `togsim_hmi/static/vendor`) |

Changes made to third-party URDF content: meshes are referenced from `togsim_description`; inertial properties, a 5th "tilt" axis,
the vacuum tool, and Gazebo/ros2_control blocks were added by this project.

Vendored copies (for offline use and because Gazebo Fortress cannot read glTF): `src/togsim_gazebo/models/{enclosure,conveyor_block,tray}_rmf`
(Open-RMF, CC BY 4.0, converted from .glb to .obj with trimesh) and `src/togsim_gazebo/models/product_bar/meshes` (Google Scanned Objects, CC BY 4.0).
