. devel/setup.bash
roslaunch traj_planner swarmtest.launch 2> >(grep -v 'TF_REPEATED_DATA')
