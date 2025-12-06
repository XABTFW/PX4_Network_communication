px4_add_module(
	MODULE modules__swarm_node
	MAIN swarm_node
	SRCS
		swarm_node.cpp
		position_sharing.cpp
		apf_controller.cpp
		velocity_obstacle_controller.cpp
		formation_planner.cpp
	DEPENDS
		px4_work_queue
	)

