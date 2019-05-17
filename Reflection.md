# CarND-Path-Planning-Project - Reflection

Self-Driving Car Engineer Nanodegree Program

## Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

## Setup

The code is split into the following modules:
- `main.cpp`: responsible for communication with the simulator -> receiving and sending events to control the car
- `trajectory.cpp`: responsible for creating trajectories and calculate costs associated with this.
- `helpers.cpp`: other util functions
- `plot.cpp`: resonsible for plotting the trajectory. Can be disabled with var `ENABLE_PLOTTING`

## Trajectory generation

This class can calculate different trajectories for different events, which are:
- KEEP_LANE
- LANE_CHANGE_LEFT
- LANGE_CHANGE_RIGHT

Also it will calculate costs for each trajectory that can be used for choosing the best trajectory

## Discussion

The issues I have experienced during the project are:
- Calculating the speed. Initially I derived the speed from the trajectory that I got back from the simulator. This didn't work out very well, so I changed this by keeping my own reference velocity.
- I implemented a plotting class to plot a global and detail of the trajectory. This gave a lot of insight.
- Currently I experience acceleration and jurk issues when changing lanes. This should be improved. Please advise how to do this. Thx

## Improvements

- Calculate more advanced trajectories, e.g. multiple scenarios for a CHANGE_LANE_RIGHT event
- Improve costs functions when more trajectories are calculated


