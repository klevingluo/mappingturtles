define topurg ranger 
(
  sensor
  ( 			
    range [ 0.0  30.0 ]
    fov 270.25
    samples 1081
  )

  # generic model properties
  color "black"
  size [ 0.050 0.050 0.100 ]
)

define erratic position 
(
  # cheating to get rid of collisions in simulation
  # size [0.1 0.1 0.1]
  size [0.350 0.350 0.250]
  origin [-0.050 0.000 0.000 0.000]
  gui_nose 1
  drive "diff"
  topurg(pose [ 0.050 0.000 0.000 0.000 ])

	# Estimated mass in KG
	mass 40.0 

	# Body shape:
  #	block
  #	( 
  #		points 8
  #		point[0] [-0.18 0.313]
  #		point[1] [0.18 0.313]
  #		point[2] [0.2525 0.18]
  #		point[3] [0.2525 -0.18]
  #		point[4] [0.18 -0.313]
  #		point[5] [-0.18 -0.313]
  #		point[6] [-0.2525 -0.18]
  #		point[7] [-0.2525 0.18]
  #		z [0 1]
  #	)
)
