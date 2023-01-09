# MatLab-Simulation-of-Serial-and-Parallel-Robots
This repository contains MatLab simulations of inverse kinematics for a 5 DoF serial robot arm and a 3 DoF parallel robot. It contains trajectory and obstacle avoidance examples for the serial robot as well as forward kinematics and workspace plotting. It contains workspace plotting for the parallel robot. Instructions to run each file are provided below.

## FINN_FK_LynxMotionArm.m - FORWARD KINEMATICS
Import to matlab and click run. To change the input angles, the user can edit the angle arrays on lines 88-92.

## FINN_IK_LynxMotionArm.m
Import to matlab and click run. To change the input end effector position, the user can edit the cartesian coordinates on lines 16-18.

## FINN_TASK_LynxMotionArm.m
Import to matlab and click run. To change the input task, the user can edit the points to plot array on line 21 by adding new points in the format x, y, z.

## FINN_TASK_LIN_TRAJECTORIES_LynxMotionArm.m
Import to matlab and click run. To change the input task, the user can edit the points to plot array on line 21 by adding new points in the format x, y, z. robot operating frequency can be changed on line 20.

## FINN_TASK_FREE_TRAJECTORIES_LynxMotionArm.m
Import to matlab and click run. To change the input task, the user can edit the points to plot array on line 21 by adding new points in the format x, y, z. robot operating frequency can be changed on line 180.

## FINN_TASK_OBSTACLE.m
Import to matlab and click run. To change the input task, the user can edit the points to plot array on line 57 by adding new points in the format x, y, z. robot operating frequency can be changed on line 46. Obstacle position and shape can be changed on lines 35-42, "i" stands for initial, "f" stands for final. 

## FINN_IK_PARALLEL_BOT.m
Import to matlab and click run. To change the input end effector position and orientation, the user can edit lines 27-29.

## FINN_WS_PARALLEL_BOT.m
Import to matlab and click run. To change the input end effector position and orientation, the user can edit lines 27-29.
