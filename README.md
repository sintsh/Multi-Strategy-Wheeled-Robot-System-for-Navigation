# Multi-Strategy-Wheeled-Robot-System-for-Navigation

Prepared by
 1. Sintayehu Zekarias
 2. Fikir Awoke

## Project structure

 The under workspace we have three projects
 1. m2wr_description - holds our car model
 2. my_worlds - holds different type of gazebo worlds
 3. motion_plan - holds  an algorithm to go from a point to another,  obstacle avoidance  and A* search
 

 		
 		 

## To launch the model 

To launch the model, open new terminal and go to simulation_ws

```
catkin_make
```

```
source devel/setup.bash
```

```
roslaunch my_worlds world1.launch
```


```
roslaunch my_worlds world2.launch
```

## To run scripts

To move car from one point to another , open terminal and go to simulation_ws
```
source devel/setup.bash
```

```
rosrun motion_plan go_to_point.py
```
To check a car detects an obstacle , open terminal and go to simulation_ws


```
rosrun motion_plan obstacle_avoidance.py
```

To move car from one point to another using A* algorithm , open terminal and go to simulation_ws


```
rosrun motion_plan move_by_astar.py
```
