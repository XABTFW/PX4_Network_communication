px4_add_module(
  MODULE drivers__imu__damiao__dmimu_l1
  MAIN dm_imu_l1
  COMPILE_FLAGS
    -O2
  SRCS
    dm_imu_l1.cpp
    dm_imu_l1_main.cpp
  DEPENDS
    px4_work_queue
)
