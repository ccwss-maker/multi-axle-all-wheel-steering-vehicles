# multi-axle-all-wheel-steering-vehicles
The program is used to simulate the effect of deflection correction in tunnels with a multi-axle independently steered transport vehicle split-vector deflection model.
Emulation environment: Ubuntu 18.04/20.04; Ros Melodic/Noetic.
Library: Opencv4.5.5; yaml-cpp0.7.0.

If all goes well, you can perform the following steps,

1.install the ROS control plugin,

sudo apt-get install ros*controller*
![图片](https://user-images.githubusercontent.com/75433402/227500200-c0447569-a87c-487d-ba12-97d1ba7b94f4.png)

2.Copy all files under /src to ROS workspace,
![图片](https://user-images.githubusercontent.com/75433402/227499846-415d7cba-4768-4ee3-b5d1-34c13c977928.png)

3.Open and change the file in the following path, catkin_ws/src/robot/robot1_program/src/car.cpp,

Change "/home/ccwss/PersonalData/Program/Ros/car5_ws" in config_yaml_path, output_data_path and output_img_path to the absolute path of your Ros workspace
![图片](https://user-images.githubusercontent.com/75433402/227510996-c0113dbe-319d-4cc7-bae3-3a6ab5cde4fc.png)

4.Compile the program,

catkin_make
![图片](https://user-images.githubusercontent.com/75433402/227500587-767fe2ee-1435-49a8-9c49-86713ed194c7.png)

5.Add setup.bash to .bashrc,

sudo gedit ~/.bashrc
![图片](https://user-images.githubusercontent.com/75433402/227512382-6a4eb7d7-f29f-44dd-8926-328b8df1b1fb.png)
Add source absolute_path/catkin_ws/devel/setup.bash to the last line,
![图片](https://user-images.githubusercontent.com/75433402/227507220-9d446103-9f83-4040-a7ed-9dff785becea.png)
Then, Execute the following command, source ~/.bashrc,
![图片](https://user-images.githubusercontent.com/75433402/227507902-dbc2d441-b90b-470f-9572-5c5d4ab1d56c.png)

6.Execute the following command to open Gazebo, 

roslaunch robot1_gazebo truck.launch
![图片](https://user-images.githubusercontent.com/75433402/227508339-f00e1126-ba2f-41a5-8661-4ec65be3d4da.png)

7.Create a new command window,and execute the following command to run the control program,

rosrun robot1 robot

![图片](https://user-images.githubusercontent.com/75433402/227508947-a7056f21-ffd5-4310-8822-ff3d9286ac41.png)
You will see nothing happen because Gazebo is paused,you can turn it on in the lower left corner of its control panel.
![图片](https://user-images.githubusercontent.com/75433402/227509522-120f9dcf-8cbe-4f59-8a5c-6738e8bf002c.png)
You will see two windows showing the wheelset steering center and the body geometry center trajectory respectively.
![图片](https://user-images.githubusercontent.com/75433402/227509673-dd4c0671-63e6-4b95-8c77-aa9896571004.png)
You can use the mouse to manipulate the image, scroll the wheel to zoom in and out, press the wheel to restore the initial scale and center the image, right click and drag to move the image position, left click to save the image to output_img_path.

8.The file in the following path can be adjusted for each direction of the deflection speed scale factor, src/robot/robot1_program/config/size.yaml,
![图片](https://user-images.githubusercontent.com/75433402/227511128-01ddac68-89c3-4f79-88d4-73fd878ff41d.png)

9.Change the body attitude at will, you will see the body deflection angle and longitudinal deflection distance automatically corrected.

