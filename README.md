# Jink.jl

Julia package for differential multi-task inverse kinematics

## Features
* This package solves multi-task inverse kinematics by composing the multiple motion tasks as a single Quadratic Program and solving it using [OSQP](https://osqp.org/) via the [JuMP.jl](https://github.com/jump-dev/JuMP.jl) mathematical programming interface.

* Jink.jl is designed to have a general API that allows you to easily adapt it to your robot of choice. All you have to do is to create a Julia sub-module for your robot that exports all desired kinematics as functions.As examples, this package provides sub-modules for the [PickleRick](https://github.com/adubredu/PickleRick.jl) planar humanoid robot and the [Digit](https://github.com/adubredu/DigitVisualizer.jl) robot.

## Installation
1. Open your Julia REPL by typing  `julia` in your terminal.
2. Press `]` on your keyboard to enter the package manager
3. Enter command `add https://github.com/adubredu/MeshCatMechanisms.jl` and press `Enter` on your keyboard to install the MeshCatMechanisms dependency
4. Enter command `add https://github.com/adubredu/RigidBodyDynamics.jl` and press `Enter` on your keyboard to install the RigidBodyDynamics dependency
5. Enter command `add https://github.com/adubredu/DigitVisualizer.jl` and press `Enter` on your keyboard to install the DigitVisualizer dependency. 
7. Enter command `add https://github.com/adubredu/Jink.jl` and press `Enter` on your keyboard to install this package.
8. Press the `Backspace` key on your keyboard to return to the REPL

## Usage
See the [examples](examples) folder for usage examples. 

## Acknowledgement
This package was inspired by [Stephane Caron](https://github.com/stephane-caron)'s [pink](https://github.com/tasts-robots/pink) python package