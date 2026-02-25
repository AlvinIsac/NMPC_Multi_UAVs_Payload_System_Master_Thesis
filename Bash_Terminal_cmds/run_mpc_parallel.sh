#!/bin/bash

# Multi-core MPC with optimized threading
export OMP_NUM_THREADS=5
export OPENBLAS_NUM_THREADS=5
export OMP_DYNAMIC=false
export OMP_PROC_BIND=true
export OMP_PLACES=cores

# Force single-threaded libraries to use multithreading
export BLAS_NUM_THREADS=5
export LAPACK_NUM_THREADS=5
export MKL_NUM_THREADS=5

# OpenBLAS specific optimizations
export OPENBLAS_CORETYPE=ARMV8
export OPENBLAS_THREAD_TIMEOUT=1000

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Launch MPC controller
ros2 run mission_handler mpc_uav1_hover_node | tee mpc_output.txt

# ros2 run mission_handler mpc_main_30_states_node | tee mpc_output_30_.txt

# ros2 run mission_handler mpc_main_30_states_with_tension_node | tee mpc_output_tension.txt