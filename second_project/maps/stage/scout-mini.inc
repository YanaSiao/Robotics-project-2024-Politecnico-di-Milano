define kinect ranger
(
  sensor
  (
    range_max 8.0
    fov 360.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define scout-mini position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.2 0.2 0.0 0.3 ]

  size [ 0.612 0.580 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "grey"
  
  kinect()
)
