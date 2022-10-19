**arena_camera**

arena_camera node publishes image data collected from Lucid Vision Labs Triton
GigE cameras in /lucid_vision/camera_"X"/image_raw topic as a ROS message(
sensor_msgs/Image). This node can connect to multiple camera devices discovered
by Lucid Vision Labs' ArenaSDK. In order to use this node, you need to install
Lucid Vision Labs' ArenaSDK.

### Installation

1. Download ArenaSDK from [here](https://thinklucid.com/downloads-hub/).
2. Install ArenaSDK.
3. Before connecting to the camera you need to set your IP address, according to
   documentation of Triton Cameras your IP address can be set to 169.254.0.1.

4. Clone this driver to <your_workspace>/src/.

5. Check Serial number of devices and change the serial number values in
   parameter files.
   (Serial number writes on the box of the camera) Serial number is different
   for each product.

   If you have only one camera work on this param file :
   <your_ws>/src/arena_camera/param/lucid_vision_camera.param.yml If you have
   more than one camera work on this param file :
   <your_ws>/src/arena_camera/param/multi_camera.param.yml

6. Build your code with following command.

   `colcon build `

7. Source the directory and run the executable with following command.

   `ros2 run arena_camera arena_camera_node_exe --ros-args --params-file <your_workspace>/src/arena_camera/param/<your_param_file>.param.yaml `

   7.1 You can check whether data is flowing or not and what is the rate of it
   with following commands.

	`ros2 topic hz /lucid_vision/<your_camera>/image_raw`

8. Open another terminal and run Rviz2 with following command.
   `rviz2`

   8.1 Add the /lucid_vision/<your_camera>/image_raw topic to Display panel.
   With following "Add">"By topic" section.

   8.2 Set your "Fixed Frame" as "lucid_vision".

   8.3 Set your "Reliability Policy" to "Best Efford".  (Best efford works in
   UDP, Reliable works in TCP/IP)

   8.4 Be sure that visibility button checked.

