Be sure to:
ln -s path_to_your_github_repo/my_pf ~/catkin_ws/src
ln -s path_to_your_github_repo/hector_slam ~/catkin_ws/src
ln -s path_to_your_github_repo/geomtery ~/catkin_ws/src

/home/jasper/comprobo2014/src


In ~/catkin_ws/src, run:
catkin_make
catkin_make install

To use localize robot on map:

roslaunch neato_2dnav amcl_builtin.launch map_file:=/home/jasper/catkin_ws/src/hector_slam/hector_mapping/maps/newmap.yaml ($PATHOFYAMLFILE)

Set frame to map on Rviz.

rosrun my_pf pf.py

Add a topic to listen to.
Select /particle, whose type is Pose. This is the mean of all particles predicted by pf.py in my_pf.

