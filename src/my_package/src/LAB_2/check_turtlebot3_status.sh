# check and display active nodes
echo "Active ROS Nodes:"
rosnode list
echo ""

# check and display active topics
echo "Active ROS Topics:"
rostopic list
echo ""

# check and display the content of /scan topic for 1 scan
echo "Content of /scan topic for 1 scan:"
rostopic echo /scan -n 1

