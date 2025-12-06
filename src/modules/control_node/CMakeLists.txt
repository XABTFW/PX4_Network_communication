px4_add_module(
	MODULE modules__control_node
	MAIN control_node
	SRCS
		control_node.cpp
		control_instance.cpp
	DEPENDS
		px4_work_queue
	)
	
