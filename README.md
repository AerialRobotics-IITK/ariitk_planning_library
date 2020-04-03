# ariitk_planning_library

Planning library for MAVs

This repository contains two packages:

skeleton_global_planner: An online planner based on the voxblox_skeleton_planner, uses mav_local_planner for trajectory execution.
frontier_explorer: A package for frontier-based exploration in unknown environments.

Very heavily based off the following repositories:

[ethz-asl/mav_voxblox_planning](https://www.github.com/ethz-asl/mav_voxblox_planning): MAV planning tools using voxblox as the map representation.

[ethz-asl/mav_active_3d_planning](https://www.github.com/ethz-asl/mav_active_3d_planning): A modular framework for online informative path planner (IPP) design.
