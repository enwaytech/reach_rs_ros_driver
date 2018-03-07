# Emlid Reach RS ROS driver

This ROS driver for the Emlid Reach RS GNSS RTK device receives NMEA sentences over the network and publishes sensor_msgs/NavSatFix messages. The NMEA sentence parsing code is based on the [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) package by Eric Perko and Steven Martin.

In comparison to similar packages (such as [nmea_tcp_driver](https://github.com/CearLab/nmea_tcp_driver)) we added a couple of features that made it more reliable to use and easier to fuse position data with other sensor data:

- automatic reconnection behaviour after connection is lost
- use the diagnostics system of ROS to provide status information
- ability to specifiy a covariance matrix for the position (especially useful when fusing sensor data)

# Configuration of the Reach RS

Configuration of the Reach RS can be done using the ReachView web interface.

On the _Position output_ tab under _Output1_ choose _TCP_. Set the role to _Server_, the address to _localhost_ and choose a port number. From the formats select _NMEA_.

The IP address of the Reach RS can be found in the top-left corner of the ReachView web interface. Note IP address and port number in order to configure the ROS driver node correctly.

# ROS driver configuration

## Published topics

- `~fix`: The GNSS fix ([nav_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))
- `~vel`: Reported velocity (only available if RMC messages are received)
- `~time_reference`: Time reference (unused)
- `/diagnostics`: Diagnostics about the hardware driver (see http://wiki.ros.org/diagnostics)

## Parameters

- `reach_rs_host_or_ip`: The host of the Reach RS device. This can be an IP address or a hostname like _reach.local_ (which is the default for the Reach RS).
- `reach_rs_port`: The network port at the Reach RS where to listen to NMEA sentences.
- `reach_rs_frame_id`: The frame_id of the published sensor_msgs/NavSatFix messages.
- `fix_timeout`: The duration in seconds after which the fix is considered to be lost. This should be set higher than the period of the publishing frequency of the Reach RS. Defaults to 0.5 seconds.
- `time_ref_source`: (unused)
- `useRMC`: Enables/disables usage of RMC sentences (default: false).
- `covariance_matrix`: Covariance matrix for the GNSS fix position (9 element array) that is published along with every nav_msgs/NavSatFix message. If not specified the covariance is estimated using the HDOP value.

