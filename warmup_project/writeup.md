# CompRobo Warmup Project
##### Kim Asenbec, Brenna Manning, Jordan Van Duyne

For this project, we have programmed a Neato robot to perform a few different tasks.
These included “Teleop Twist”, driving the neato in a square, and following a wall.


### Writeup
- For each behavior, describe the problem at a high-level.  Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

- For the finite state controller, what was the overall behavior.  What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.

- How was your code structured?  Make sure to include a sufficient detail about the object-oriented structure you used for your project.

- What if any challenges did you face along the way?

- What would you do to improve your project if you had more time?

- What are the key takeaways from this assignment for future robotic programming projects?


# Teleop Twist
In this task, we wrote our own code to teleoperate the robot. Our code is meant to mimic the behavior of teleop_twist_keyboard. To determine the appropriate twist for each keystroke, we ran “rostopic pub /cmd_vel Twist” from the command line, while running teleop_twist_keyboard. Doing this allowed us to observe the linear and angular values tied to each keystroke within teleop_twist_keyboard.
We structured our code within a TeleopNode class. After detecting a keystroke, we enter the corresponding conditional statement. Within each condition, we set the appropriate values for the twist. The twist is then published, and the neato moves as directed.

# Drive Square
For this task, we wrote code to make the robot drive in a 1m by 1m square path using timing. Similarly to the Teleop Twist task, we published a twist using  /cmd_vel Twist to make the robot move.  This code was structured within a SquareNode class, and switched between two states: “forward” and “turning”. In the forward state, the robot is sent a twist with a positive linear vector of (1.0, 0,0, 0.0) so that it moves forward.  Using this twist, the robot will drive forward at 1 m/s, so after 1 second has passed it will have driven 1 meter. After 1 second has passed in the driving forward state, the state switches to the turning state. Here, the robot is sent a twist with an angular vector of (0.0, 0.0, 0.5) which causes it to rotate left at a speed of 0.5 radians per second. After approximately pi seconds, a full 90 degree turn is completed, and the state is switched back to the forward state. This repeats until the robot has fully driven in a square. 


# Wall following

The goal of the wall following behavior is to enable a Neato situated near a wall to drive forward and parallel along that nearest wall.

## Detecting nearest wall
In order to drive parallel to the nearest wall, the nearest wall first has to be identified: where is this wall relative to the Neato?

![closest corner](imgs/closest_corner.png)

We use the Lidar laser sensor to determine the range distance between each of the Neato's four corners (corresponding to 45˚, 135˚, 225˚, and 315˚ of the laser) and any obstacle in those directions. As shown in the picture above, we will assume that the corner associated with the smallest range is the corner that is closest to a wall.

## Aligning with the wall
In order for the Neato to be considered parallel with the wall, both range distance readings from the side (left or right) of the closest corner have to be within a certain threshold of each other. For example, if the back right corner is considered to be the closest corner and has a range reading of 0.9 m, the front right corner should also have a range reading of 0.9 m. This state is shown in the picture below.

![parallel](imgs/parallel.png)

However, due to difficulty with timing and not infinitely accurate sensors, the two readings from the same side will almost never be exactly the same. Thus, we consider them to be close enough for the Neato to be parallel if their difference is less than .25 m. This threshold was determined through trial and error. Smaller thresholds caused the Neato to spend too much time aligning itself with the wall due to not being considered parallel, while larger thresholds caused alignments that were obviously skewed to be considered parallel.

If the difference of the two readings is within the threshold, then great! The Neato is parallel to the wall and can move forward. However, if the readings are too far apart, the Neato needs to turn to align itself with the wall. The direction in which it needs to turn depends on which corner is closest to the wall and is shown in the diagrams below. The Neato has achieved parallel alignment once it has turned enough that the difference between the two readings is within the threshold.

![back right: turn right](imgs/back_right.png)
![front right: turn left](imgs/front_right.png)
![back left: turn left](imgs/back_left.png)
![front left: turn right](imgs/front_left.png)

## Implementation
In order to achieve this functionality, we subscribe to the `/scan LaserScan` message in order to find the laser's readings. We have a callback `WallFollower.process_scan(self, m)` that saves the ranges associated with the four corners every time a `/scan` message is published.

At the same time, we are constantly running a `while` loop that does all of the computation necessary to determine how the Neato should act. As long as range readings are available (meaning `WallFollower.process_scan(self, m)` has been called at least once), the closest corner is identified by comparing the saved scan readings. Then, the parallel test is performed (checking if the readings from the closest corner and the other corner on the same side are close enough) and the state of parallel-ness is set to either `True` or `False`. If the Neato is parallel, then a `/cmd_vel Twist` message object is created with a forward (linear x) velocity of 1.0 m/s. If The Neato is not parallel, then a `/cmd_vel Twist` message object is created with an angular (z) velocity of ±0.4 radians/s to turn the Neato in the appropriate direction to help it achieve parallel-ness. Then, after the correct Twist object is created, the `/cmd_vel Twist` message is published. Then the loop starts over from the beginning!

## Decisions and challenges
The first decision we had to make was having the Neato repeatedly check if it is parallel to the wall. If we had simply allowed it to move forward indefinitely as soon as it was first calculated to be parallel, it could easily stray from the wall if it was even slightly skewed before it started going forward.

Determining the correct turn velocity and the correct parallel threshold values was also difficult. Too fast of a velocity results in the Neato overshooting the parallel state between range updates, causing the Neato be stuck in a "twitching back and forth" motion. Too slow a speed and the Neato is just too slow to behave nicely. A small range threshold, similar to too fast of an angular velocity, results in a twitch motion because falling within the range is too difficult. Too large a range and what the program considers parallel is actually very skewed.

// a large challenge we faced was dealing with updating the neato's speed based on the newest laser readings; it is possible the readings might not have come in yet.
// HOWEVER why did we even need a run function; why didn't we put all of this in the callback..? Something to think about.
