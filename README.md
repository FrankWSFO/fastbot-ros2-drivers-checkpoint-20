To bringup a fastbot.

For Arduino files, see Josh Newan's fork of ros_arduino_bridge: https://github.com/joshnewans/ros_arduino_bridge.git.

To be able to transmit topics between the real fastbot and a rosject, on fastbot, execute:

	export ROS_DOMAIN_ID=1

and on rosject terminal, execute:

	export ROS_DOMAIN_ID=1
	export CYCLONEDDS_URI=file:///var/lib/theconstruct.rrl/cyclonedds.xml
	ros2 daemon stop 
	ros2 daemon start
	ros2 topic list

