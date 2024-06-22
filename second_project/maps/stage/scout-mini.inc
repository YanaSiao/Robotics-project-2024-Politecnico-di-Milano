define lidar ranger
(
  sensor
  (
    size [ 0.06 0.15 0.03 ]
    range [0 50]
    fov 360.0
    samples 1440
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define scout-mini position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "odom"

  odom_error [0.01 0.01 0.0 0.01 ]

  size [ 0.618 0.580 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "grey"

  velocity_limits [ 1.0 1.0 1.0 ]  # Max linear velocity (x, y) and angular velocity (z)
  acceleration_limits [ 0.5 0.5 1.0 ]  # Max linear acceleration (x, y) and angular acceleration (z)

  lidar(pose [ -0.1 0.0 -0.11 0.0 ])
)