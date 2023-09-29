# MazeRover

## Warning
This code was all written during a very sleep-deprived three-week period, with constant iteration of the code and the robot. Therefore it is pretty scrappy. There was also some starter code (and a simulator) provided by the teaching team, however, I have removed all of that before uploading it to this repo as it is not my code, and isn't really used. Even though this project is still assigned to students, I have chosen not to redact any code as:
 1. In order to run it you need our specific robot with its specific wiring setup
 2. The likelihood that someone uses this repo to cheat is very low
 3. I myself cannot even make sense of this code and I wrote (most of) it

The main file is test.m

## Overview
As part of a mechatronics class, myself and three other friends created a robot capable of navigating a maze. More specifically, the robot had to:

 1. Drive around the maze and localize itself within it
 2. Once localized, drive to a predetermined 'pickup' location
 3. At the pickup location, detect and pick up a block
 4. Drive the block to a 'dropoff' zone, and drop it off

The robot had to be built totally from scratch, and we were graded on its performance across three milestones over the course of three weeks. While all four of us contributed to all aspects of the rover, we each had different primary responsibilities, mine being the MATLAB code that controlled the rover inside of the maze, and sent commands to the onboard microcontrollers.


## Rover Control Strategy
The rover's control strategy was based on input from seven ultrasonic sensors, as seen in the diagram below. The grey "5" and "6" sensor positions indicate the original position of those sensors before they were shifted to face the sides of the robot. Note that sensors 1 and 7 are on the front of the robot, with 7 being mounted on the undercarriage to allow for the detection of the block.
![sensor layout](https://github.com/WFERRIE/MazeRover/assets/58156317/cff967a8-d1d3-43ef-942a-1f7cc713814b)

On each iteration of the robot's control algorithm, the robot would read from each sensor, allowing it to detect the presence of nearby walls, and therefore perform localization and navigate the maze. The sensor pairs 5/2 and 6/4 also allowed the robot to calculate its angle relative to the walls surrounding it.

## Localization Strategy
Localization of the robot was done using a 2D localization strategy. As the maze layout was known ahead of time, the rover would read from its sensors, and depending on the presence or lack of walls on each side, would refine a probability map of its location in the maze. Once the probability values met a certain threshold, the rover indicated via a flashing light that it was localized, and followed a "map" of the maze to the pickup location.

## Block Delivery Strategy
Once the rover arrived in the pickup location, it would open its gripper, exposing the undercarriage sensor (sensor #7). The robot would then scan a 180° field-of-view in 5° increments searching for large changes in distance readings, indicating a block. If no block was found the robot would move further into the pickup location and repeat. If the block was found, the robot would hone in on the block's location, constantly scanning the environment ahead of it until the block was located within the robot's gripper. The gripper would then close, and the robot would drive to the drop-off zone. 

## Final Result
The Robot performed well and was able to complete all tasks! Below is a picture of our beautiful baby:
![darnell](https://github.com/WFERRIE/MazeRover/assets/58156317/10862090-085b-4273-a338-7f44a072f27c)
