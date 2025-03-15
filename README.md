# Solve Maze - TurtleBot3 Maze Solving

## Project Overview
This project is developed to solve a maze using **TurtleBot3 in the Gazebo simulation environment**. TurtleBot3 receives **laser measurements via /scan messages** and **movement commands via /cmd_vel messages**. Using the **Wall Following algorithm**, TurtleBot3 attempts to navigate through the maze and reach the central point.

## Installation
Follow these steps to set up the project on your system.

### 1. Clone the Repository
```bash
cd ~/robotlar_ws/src
git clone https://gitlab.com/blm6191_2425b_tai/blm6191/micromouse_maze.git
```

### 2. Compile Dependencies and Update Environment
```bash
cd ~/robotlar_ws
catkin_make
source ~/.bashrc

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

# Set Gazebo model path
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/robotlar_ws/src/" >> ~/.bashrc
source ~/.bashrc
```

### 3. Launch the Maze Simulation
```bash
roslaunch micromouse_maze micromouse_maze4.launch use_sim_time:=false
```

## Solver.cpp - Maze Solving Algorithm
In this step, we will create the **solver.cpp** file, which enables TurtleBot3 to navigate through the maze. The **Wall Following algorithm** is implemented so that the robot follows walls and reaches the central point.

### Define Solver.cpp and Compile
Add the following lines to your **CMakeLists.txt** file to make the solver.cpp file compilable:
```cmake
add_executable(my_solver src/solver.cpp)
target_link_libraries(my_solver ${catkin_LIBRARIES})
```

### Compile and Update Environment
After every change, compile the code using the following commands:
```bash
rosnode kill -a   
pkill -9 gzserver
pkill -9 gzclient
pkill -9 roscore
cd ~/robotlar_ws
catkin_make
source ~/.bashrc
```

### Run the Solver
```bash
rosrun solve_maze my_solver
```

This command executes **solver.cpp**, allowing TurtleBot3 to start solving the maze.

## Camera Adjustment
In Gazebo, adjust the camera **relative to TurtleBot3** for better observation of the simulation.

## Shutting Down the Simulation
To terminate the simulation, run the following commands:
```bash
rosnode kill -a   
pkill -9 gzserver
pkill -9 gzclient
pkill -9 roscore  
```

## Results
In this project, the **Wall Following algorithm** is implemented to solve the maze using TurtleBot3. The robot utilizes **laser data from /scan messages** to follow the nearest wall and reach the center of the maze.

Additionally, **/odom messages** are used to track the robot's position. The developed ROS package is tested using **micromouse_maze4.launch**.

---

This README is prepared according to the requirements of **Question 1**.

