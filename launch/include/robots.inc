define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  #size [ 0.2552 0.2552 0.40 ]
  size [ 0.3 0.3 0.10 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
)

define turtlebot_gps position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "gps"
      localization_origin [0 0 0 0]

  #size [ 0.2552 0.2552 0.40 ]
  size [ 0.3 0.3 0.10 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
)

define roomba position
(
  size [0.33 0.33 0.1]

  # this block approximates the circular shape of a Roomba
  block( 
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 0.1]
  )

  # odometry error model parameters, 
  # only used if localization is set to "odom"
  odom_error [0.03 0.03 0.00 0.05]

  color "gray50"
)

define roomba_gps position
(
  size [0.33 0.33 0.1]

  # this block approximates the circular shape of a Roomba
  block( 
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 0.1]
  )

  localization "gps"
      localization_origin [0 0 0 0]

  color "gray50"
  

)

