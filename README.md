# ros-system-daemon
This package provides functionality for automatically starting and stopping ROS, including a facility
for re-launching the daemon job as your own user.

## ROS System User
    Username:           ros
    Home Directory:     /var/lib/ros
    Group:              ros
    Shell:              /bin/sh

*ros* should be a member of group *dialout* in order to access serial ports.

```chown -R ros:ros /var/lib/ros```  
```chmod 2775 /var/lib/ros```

    ROS Log Dir:        /var/log/ros
    ROS PID:            /var/run/ros/roscore.pid

## Startup and shutdown
Startup and shutdown is controlled by an upstart job. By default, ROS is launched as user `ros` with the following setup:

    source /etc/ros/setup.sh
    source /etc/ros/envvars
    export ROS_HOSTNAME=${HOSTNAME}
    roslaunch /etc/ros/robot.launch
    
The files `/etc/ros/setup.sh` and `/etc/ros/robot.launch` are symlinks controlled by update-alternatives.
You can load setup scripts and launchers like so:

    update-alternatives --install /etc/ros/setup.sh ros-setup ${HOME}/my_catkin_ws/devel/setup.sh 20
    update-alternatives --install /etc/ros/robot.launch ros-robot-launch ${HOME}/my_robot.launch 20

You can view what scripts and launchers are available with:

    update-alternatives --list ros-setup
    update-alternatives --list ros-robot-launch

And change the currently selected alternave with:

    update-alternatives --config ros-setup
    update-alternatives --config ros-robot-launch

## System-wide Logging
`ROS_LOG_DIR=/var/log/ros` for the background daemon job. This path is owned by the ros user and group.

### ros-system-daemon.ros.logrotate
* Config file ```/etc/logrotate.d/ros```
* Rotate logs daily *.log -> *.log.1 -> *.log.2 -> etc
* Compress the previous days logs daily *.log.2 -> *.log.2.gz
* Keep up to one weeks logs for an active rosmaster
* Archive inactive log subdirectories daily
* Remove archived logs older than one week
* Rotation currently done by copying and truncating the active log

Note: 01 Jan 2013 Sending SIGHUP to ROS does not cause it to write to a new log.  
<https://github.com/ros/ros_comm/issues/45>

## Files
### /usr/sbin/rosctl
Modelled after ```apachectl```, this script allows ros to be started
locally by users or system-wide by user *ros*.

If ```rosctl``` is run as user *ros* it will attempt to launch ```/etc/ros/robot.launch``` or the launch file specified in ```ROS_LAUNCH``` if it exists.

If ```gnome-session``` is running, the script will attempt to send desktop notifications via ```notify-send``` to the user that is logged in when ROS starts or stops.

Sub-commands
* start    Start ROS (roscore + default launch file)
* stop     Stop ROS (rosnode kill nodes then killall roslaunch)
* restart  Start followed by Stop
* status   Display the PID and user running ROS

In the near future running this as script as root will check/repair directory permissions and re-run itself setuidgid ros. Also, when run as root it should issue a warning and run ```initctl stop ros``` to prevent upstart from respawning. It may also be worth issuing a warning when run as *ros* and ```initctl status ros``` shows that upstart will respawn the process.

### /etc/ros/envvars
The intent is that may be parsed and edited by hardware vendor install scripts; a place to put customizations which does not require editing setup.bash.

## Special Thanks
The program ```rosctl``` was inspired by ```apachectl```
Package was influenced by the alternate approach used by
Thomas Moulard on ros_comm_upstart
Bill Morris on Turtlebot-Mfg
