define kinect ranger
(
  sensor
  (
    size [ 0.06 0.15 0.03 ]
    range [0 8]
    fov 360.0
    samples 1440
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define scout-mini position
(
  pose [ 1.0 1.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.001 0.001 0.0 0.001 ]

  size [ 0.612 0.580 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "grey"
  
  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
)
