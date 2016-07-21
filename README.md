# fotonic_3dcamera
ROS driver for the Fotonic E-Series 3D cameras

# Available params

* ip_adress: IP adress of the camera
* fotonic_frame_id: frame_id of the camera

Sensor parameters:
* sensor_shutter_ms: shutter time
* sensor_framerate: desired framerate
* sensor_framerate_divisor: sets a divisor to the FPS value, so that the camera outputs images at a lower rate, but runs the higher frame rate internally.

Hardware filters:
* lerp_filter_enable: enables LERP filter
* edge_filter_enable: enables edge filter
* edge_filter_minB: 
* edge_filter_diff1
* edge_filter_diff2
* edge_filter_diff3

Software filters:
* active_brightness_min: filter by minimum brightness
* active_brightness_max: filter by maximum brightness
* z_range_min:  filter by minimum range
* z_range_max:  filter by maximum range
* pre_zfilterx_enable
* pre_zfiltery_enable
* pre_zfilter_depth
* noise_filter_enable
* noise_filter_radius
* noise_filter_depth
* noise_filter_pixels

# Launch the node

```
$ roslaunch fotonic_3dcamera fotonic_3dcamera.launch
```

# Topics

Published topics:
* ~/point_cloud
* ~/point_cloud2
* ~/depth/image_raw
* ~/ir/image_raw
* ~/temperature

