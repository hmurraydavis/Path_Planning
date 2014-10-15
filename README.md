###Summary and Objectives
-----
The goal for this roject was to develop code that would enable a root to plan a path through a traversable space. Idealy, this space would have obstacles in it. We chose to focus on a more algorythmically based approach as opposed to one which foccused on directing the physical robots to do things. While physical robots were involved, the main objective was to gain a deeper understanding of the core algorythms by implementing them. 

This project was the second of four progressively more involved robotics projects in Computational Robotics at Franklin W. Olin College of Engineering and was developed over 3.5 weeks. Projects focused on particle filtering, mapping, multi-agent systems, and path planning.

For this particurlar project, the algorithms behing and for path planning and localization were explored. [SLAM](http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) mapping was used to create a map or "occupancy grid" describing the area around the robot. This occupancy grid was then used as the innput map for finding the shortest path from the robot's current location to a specified goal using [Dijkstra's Algorithm](http://en.wikipedia.org/wiki/Dijkstra%27s_algorithm). Running Dijkstra's Algorythm yielded a list of waypoints. This list of waypoints was then traversed by the robot in the physical world. To obtain the physical location of the robot while traversing waypoints, the robot's location is determined from SLAM mapping compared to the initially generated map as it traverses the waypoints.


-----
#### Particle Filtering
The current position and direction is determined by a particle filtering algorithm (written by Paul Ruvolo). The particle filtering algorithm continuously updates the position and direction of the robot, which are determined by comparatively weighing its lidar and odom readings to the robot's mapped environment. We assumed that the mean of the estimated positions and directions represents an accurate position and direction of the robot in a period of time.

#### Waypoint Following
Assuming that particle filtering works successfully and locations of waypoints are available, there is enough information to navigate a robot to its waypoints. What is left is simple trigonometry to direct where the robot should be directed towards and proportional control.

Below is a snippet of code that illustrates how trigonometry is used to calculate the desired heading from point A to point B in degrees. 

```
import math
delta_y = y_B - y_A
delta_x = x_B - x_A
angle = atan2(delta_y/delta_x)*180/math.pi
```

Given desired heading and current direction, we use proportional control to command the speed at which it rotates in the clockwise or counter-clockwise direction. The linear speed is set to low in order to maintain an accurate estimation of current position and direction.

#### Waypoint Check-off
The robot keeps track of its current waypoint. When it detects that it is close to its waypoint, it directs itself to the next waypoint. If there are no waypoints, the robot stays in place. 

### Design Decision/ Code Structure

We've added combined the scripts that we worked on into one directory. In addition, we established that one script would import the other and call its methods. This script was the path follower, which imported the path planner. Due to this establishment, we were able to gather information from the path planner before calling the path follower. This code flow made sense logically. At one point, we were considering having the path planner publish to a node that the path follower would subscribe to. However, the path planner takes time to run, which would likely cause problems in following waypoints for the path follower.

### Challenges

We wished we had allocated more time into integrating the code, because during that process, we came across several bugs, which were larger problems than we had expected. For instance, we wanted our path planning code to publish to a node, but to be able to publish to a node, it needed to be casted into a select amount of data types. We had originally tried to publish an array of tuples of waypoints but spent hours looking through documentation and debugging to get waypoints published. In the end, we decided on publishing a string of etc.

### Future / Interesting Lessons
We found that it is important to dedicate more time towards integration in future robotic programming projects. We had largely underestimated integrating code between the path planning system and path following system. In addition to the graphic of the estimated current position and heading, it would be great to visualize the path of the robot. That way, it would be easy to see how far off our robot is from the path and how it is deviating. 

-----
Running and Installing Code
-----
First, clone the Path_Planning repo onto your local instalation of Ubuntu 12.04 with:
```Shell
git clone https://github.com/YOUR_GITHUB_USER_NAME/Path_Planning.git
```
Alternatively, clone the repository with SSH keys or Subversion! 

Next, you'll need to establish symlinks between the working file system you just cloned and your catkin workspace. If you're new to ROS, the Catkin workspace handles running your ROS Packages. If you don't already have a Catkin workspace, a good guide to how to create one is [here](https://sites.google.com/site/comprobofall14/home/howto/setting-up-your-environment). 

Once you have a Catkin workspace, or if you have one already, create symlinks between the catkin workspace and the repository. Symlinks effectively allow the same files to exist in two different locations on your computer. Create them with:
```Shell
ln -s path_to_your_github_repo/my_pf ~/catkin_ws/src
ln -s path_to_your_github_repo/hector_slam ~/catkin_ws/src
ln -s path_to_your_github_repo/geomtery ~/catkin_ws/src
```

Now, it's time to make the catkin workspace so we can run the code with ROS!
```Shell
cd ~/catkin_ws/src
catkin_make
catkin_make install
```

To use localize robot on map:
roslaunch neato_2dnav amcl_builtin.launch map_file:=/home/jasper/catkin_ws/src/hector_slam/hector_mapping/maps/newmap.yaml ($PATHOFYAMLFILE)

Set frame to map on Rviz.

rosrun my_pf pf.py

Add a topic to listen to.
Select /particle, whose type is Pose. This is the mean of all particles predicted by pf.py in my_pf.

To allow the path planning code to receive the map generated by SLAM maping, run:
```Shell
roslaunch path_planning start_pathplanner.launch 
```
**Note: If you get an error about there not being a ROS topic about map_static, this is most likely the issue. Make sure you have started this topic. This is a distinct step from begining the ROS operating system.**

-------
Code Ideology
-------
The general idea behi
