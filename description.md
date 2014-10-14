# Path Planning
### Project Goal

The goal of our project was to plan a path for our robot and to follow it.

### How did you solve the problem?

We tackled the problem separately by dividing up the tasks into two subtasks: path planning and path following.
#### Path Planning


#### Path Following
To follow a path, all you need are its current position, current direction and waypoints.

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

### Design decision
Describe a design decision you had to make when working on your project and what you ultimately did (and why)?  These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

### Code Structure

We imported external packages.

### Challenges

We wished we had allocated more time into integrating the code, because during that process, we came across several bugs, which were larger problems than we had expected. For instance, we wanted our path planning code to publish to a node, but to be able to publish to a node, it needed to be casted into a select amount of data types. We had originally tried to publish an array of tuples of waypoints but spent hours looking through documentation and debugging to get waypoints published. In the end, we decided on publishing a string of etc.

### Future
If we had more time, we would visualize the path of the robot. At our project's current stage, we simply wanted to integrate our code.

### Did you learn any interesting lessons for future robotic programming projects?  These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
