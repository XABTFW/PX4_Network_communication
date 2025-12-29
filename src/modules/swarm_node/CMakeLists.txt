px4_add_module(
	MODULE modules__swarm_node
	MAIN swarm_node
	SRCS
		swarm_node.cpp
		position_sharing.cpp
		velocity_obstacle_controller.cpp
		formation_planner.cpp
		role_manager.cpp
		avoidance_filter.cpp
		group_coordinator.cpp
	DEPENDS
		px4_work_queue
	)

