# lanelet2_hackaton
Rebuild project on every change: <br>

``
colcon build --packages-select osm_cartography
``
<br>
<br>
On first build run:
``
source install/setup.bash
``
<br>

Setup Rviz:

1. Set the fixed frame to "map"
2. Add a MarkerArray display
3. Set the topic to "/osm_markers"
4. Add RobotModel
5. Add TF 

Run this shit:
<br>
<br>
``
ros2 run osm_cartography osm_cartography_node
``
