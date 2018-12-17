define laser ranger
(
  sensor(
    # laser-specific properties
    # factory settings for hokuyo LMS200	
    range [ 0.0  5.0 ]
    fov 270.0
    samples 270
  )
  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)

define depth ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define sonar sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ] 
  # define the range bounds [min max]
  range [0 2.0]
  # define the angular field of view in degrees
  fov 15
  # define the number of samples spread over the fov
  samples 1
)

define sonar_sensor ranger
(
 # 8 transducers spread about the robot as follows
  sonar( pose [ 0.115 0.115 0 0 ] )
 )

define sonar_array ranger
(
 # 8 transducers spread about the robot as follows
  sonar( pose [ 0.075 0.130 0 30 ] )
  sonar( pose [ 0.115 0.115 0 0 ] )
  sonar( pose [ 0.150 0.080 0 -30 ] )
 )
