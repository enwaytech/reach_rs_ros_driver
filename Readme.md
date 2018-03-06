# Emlid Reach RS ROS driver

This driver for the GNSS RTK receiver receives NMEA sentences over the network and publishes sensor_msgs/NavSatFix messages.

## Subscribed topics

None

## Published topics

- `~fix`: The GNSS fix
- `~vel`: Reported velocity (only available if RMC messages are received)
- `~time_reference`: Time reference (unused)
- `/diagnostics`: Diagnostics about the hardware driver

## Parameters

- `reach_rs_host_or_ip`: The host of the Reach RS device. This can be an IP address or a hostname like _reach.local_.
- `reach_rs_port`: The network port at the Reach RS where to listen to NMEA sentences.
- `reach_rs_frame_id`: The frame_id of the published sensor_msgs/NavSatFix messages.
- `fix_timeout`: The duration in seconds after which the fix is considered to be lost. This should be set higher than the period of the publishing frequency of the Reach RS. Defaults to 0.5 seconds.
- `time_ref_source`: (unused)
- `useRMC`: Enables/disables usage of RMC sentences (default: false).
- `covariance_matrix`: Covariance matrix for the fix (9 element array)
