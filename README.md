simple_ur
=========

A simple ROS package for talking to universal robots using the `python-urx` package.

This package right now does not support continuous servoing commands to the robot, because each command is currently a *separate program* that is uploaded to the UR5 and executed.  However, this package does provide ros interfaces for programmatically setting up robot parameters as well as TEACH mode, all the while sending out robot state info as well.  Think of this as a quick and dirty (but safe) way to use the UR5 with ROS.
