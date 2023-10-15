# 2DOFArm
Inverse Kinematics calculator to make a 2 degree of freedom arm draw a circle. 

The three main functions in this file are genCircle, solveInverse, and sendMessages.
genCircle plots the coordinate points of the full circle using both x- and y-axis based generation, solveInverse calculates the angles needed for each coordinate pair, and sendMessages turns those angles into formatted strings and sends them as mqtt packets.
