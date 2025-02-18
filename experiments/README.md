# GlobalPathInvariance

This repository contains the code for the work on global path invariance developed by Adeel Akhtar, Nan Wang, Ricardo Sanfelice, and Eric Partika

## Global_car.m

This file contains the code for controlling the vehicle. Optitrack must be set up to broadcast position data to the vehicle.

## GlobalCircle_sim.m

This file contains the simulation of the global controller for the cirlce path.

## GlobalSin_sim.m

This file contains the simulation of the global controller for the sinusoidal path.

## purepursuit.m

This file contains the pure pursuit controller that is used as \kappa_1 in the global controller. The look ahead gain will need to be tuned for diffrerent paths.

