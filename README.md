# Boustrophedon Planner
Boustrophedon Planner is a coverage path planner that implements a modified cellular decomposition algorithm. The original can be found [here](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6806237/).

## Overview
The path planner is an actionlib server that takes in a `geometry_msgs/PolygonStamped` and a `geometry_msgs/PoseStamped`,
and returns a `StripingPlan` message which contains a list of waypoints to stripe the passed in polygon.

![Sample Boustrophedon Plan](half-y-turn-concave.png)

### Instructions

run the server using:
```
roslaunch boustrophedon_server boustrophedon_server.launch
```
In its current form, the launch file requires the IKH ```aristos_maps``` package to load an actual map and run. Replace it if necessary.

In the Rviz window publish as many points as desired to form a polygon. Then, publish a pose estimate that corresponds to the robot's initial position. The orientation defined is the orientation of the produced stripes, if of course the relative flag is activated.

After the path is produced, the user is asked to input a name for this area. It is suggested to use a unique name that has some logical connection with the area. A list with the names the user has already used appears for convenience. After selecting and typing the name press enter to finalize the process and input the next polygon. The path is saved in the parameter server, from where it can be accessed or dumped in a YAML file.  

## Changelog

- January 23, 2020:
  - The boustrophedon planner can now handle all types of simple polygons that it is given, both convex and concave.
  - Inner boundaries are supported, allowing the planner to create paths at a certain offset from the initial given boundary.
  - "Half - Y" turns are supported, allowing for some curvature at the start / end of paths to assist robots that are not      completely holonomic.
  - Many new launch file parameters allow the user to specify more precisely the behavior that they want the plan to have

## License

This repository is subject to GNU General Public License version 3 or later due to its dependencies.

The geometric operations rely on CGAL which is restricted by GNU General Public License version 3 or later.
