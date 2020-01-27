# FrogBot

Some C++ code examples that formed a small part of a PhD project investigating
foot-ground interactions in animal locomotion.

Rigid-body simulations of dynamics are fast and easy to perform,
for example using by the physics library (MuJoCo)[https://www.mujoco.org]. However,
real animals and robots interact with very complex materials and substrates.
While we could model these materials within the simulation environment,
it might actually be easier to connect a force-sensing robot to a simulation,
and have it physically apply and measure contact forces in real-time with the 
simulation.

Essentially, I perfomed physics simulation with hardware-in-the-loop contact dynamics
in a new interface I called Reverse Haptics.

Read more in the paper here:
https://ieeexplore.ieee.org/abstract/document/8452298

E. A. Eberhard and C. T. Richards, “Simulation of muscle-powered jumping with hardware-in-the-loop ground interaction,” in IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), 2018.


## Contents
`ethraw` provides utilities for sending and receiving raw ethernet packets
on Mac and Linux, used for low latency interfacing of a simulation PC and
a robotic manipulator.

`mujocoToolbox` provides useful rendering and calculation utilities for the 
 physics simulation framework.

`simpipe` is the main application that runs a multi-body simulation, sends state information over ethernet
to a robot, and applies measured robot forces back into the simulation.

