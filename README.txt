About
=====
A codebase for experiments on using the extended Kalman filter to estimate link inertial parameters of some simple robots.

A summary/report documenting the more important findings with this codebase is at summary.pdf

Feel free to reach me if you have any questions:
dzhang323@hotmail.com


Instructions
============
First make sure you have the Peter Corke's robotics toolbox set up since the code depends on that.

To include the necessary paths, run startup_toy.m (note: you might have to go to its directory in Matlab for this to work).

The main script is main.m. Execute its numerous sections to setup, estimate and plot results. Various options are for the experiments are available as commented out code in the main script...

The code relies heavily on variables defined in the workspace... I apologize.

utils/initSetups.m sets up some variables that are used in the scripts.

The scripts/ directory has potentially useful scripts (and a function haha)
