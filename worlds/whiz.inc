# The Whiz Robot
define whiz_base position
(
    #color "beige"			# Default color.
    color "blue"			# Default color.
    drive "diff"		 	# Differential steering model.
    gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
    obstacle_return 1           	# Can hit things.
    ranger_return 0.5            	# reflects sonar beams
    blob_return 1               	# Seen by blobfinders
    fiducial_return 1           	# Seen as "1" fiducial finders

    localization "gps"
    localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

    # alternative odometric localization with simple error model
    # localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
    #  odom_error [ 0.05 0.05 0.1 ]   	# Odometry error or slip in X, Y and Theta
                                    # (Uniform random distribution)

    # four DOF kinematics limits
    # [ xmin xmax ymin ymax zmin zmax amin amax ]
    velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]
    acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]

    # actual size
    size [0.4 0.44 0.4] # sizes from MobileRobots' web site

    # the Whiz center of rotation is offset from its center of area ??
    origin [-0.04 0 0 0]

    # estimated mass in KG
    mass 23.0



)


define whiz whiz_base
(  
  # simplified Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555 ]

    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.4]
  )

  wheeldistance 0.413

  # whiz sick will be ranger:0
  sicklaser(
    pose [ 0.000 0.000 0.000 0.000 ]
    update_interval 100
  )
   # add a camera to the robot
    camera(
      pose [ 0.000 0.000 0.250 0.000 ]
      range [ 0.1 8.0 ]
      resolution [ 600 450 ]
      fov [ 60 45 ]
      pantilt [ 0 0 ]
      alwayson 1
      update_interval 200
    )

)

