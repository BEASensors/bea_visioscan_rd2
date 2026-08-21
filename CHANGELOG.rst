Visioscan driver for ROS2
===============================

1.1.0
-------------------------------
* Fix LaserScan angle_increment sometimes published as 0.0 (race condition reading sensor angle/skip parameters; now retried on timeout instead of using default 0 values)
* Remove fixed -90 degree offset from scan angle_min/angle_max so published angles match the sensor's reported values
* Fix build failure on ROS 2 Humble (missing header include)


0.1.2
-------------------------------
* Fix angle range issue
* Delete class rviz_common/Time for compatibility
* Delete frequency parameter in yaml file to make it more clearly


0.1.1
-------------------------------
Main features:
* MDI transmission via UDP/TCP
* Add parameter yaml file for convenient configuration
* Add launch file for quick start
