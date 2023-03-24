# multi-axle-all-wheel-steering-vehicles
The program is used to simulate the effect of deflection correction in tunnels with a multi-axle independently steered transport vehicle split-vector deflection model.
Emulation environment: Ubuntu 18.04/20.04; Ros Melodic/Noetic.
Library: Opencv4.5.5; yaml-cpp0.7.0.

If all goes well, you can perform the following steps,

1.install the ROS control plugin,

sudo apt-get install ros*controller*
![图片](https://user-images.githubusercontent.com/75433402/227500200-c0447569-a87c-487d-ba12-97d1ba7b94f4.png)

2.Copy all files under /src to ROS workspace
![图片](https://user-images.githubusercontent.com/75433402/227499846-415d7cba-4768-4ee3-b5d1-34c13c977928.png)

3.Open and change the file in the following path, catkin_ws/src/robot/robot1_program/src/car.cpp

4.Compile the program,

catkin_make
![图片](https://user-images.githubusercontent.com/75433402/227500587-767fe2ee-1435-49a8-9c49-86713ed194c7.png)




