# CompRobo Warmup Project
##### Kim Asenbeck, Brenna Manning, Jordan Van Duyne

This project was the first assignment of our computational robotics course. It provided an opportunity for us to gain experience programming robots in a team setting and to become more familiar with using ROS. The goal of this project was to program a Neato robot to perform several different tasks. These included moving as dictated by keyboard input (“Teleop Twist”), driving in a square, following a wall, following a person, avoiding obstacles, and implementing a finite state machine with behaviors of our choice.

# Teleop Twist
In this task, we wrote our own code to teleoperate the robot. Our code is meant to mimic the behavior of teleop_twist_keyboard. To determine the appropriate twist for each keystroke, we ran “rostopic pub /cmd_vel Twist” from the command line, while running teleop_twist_keyboard. Doing this allowed us to observe the linear and angular values tied to each keystroke within teleop_twist_keyboard.
We structured our code within a TeleopNode class. After detecting a keystroke, we enter the corresponding conditional statement. Within each condition, we set the appropriate values for the twist. The twist is then published, and the neato moves as directed.

# Drive Square
Our goal here was to make the Neato robot drive in a 1m by 1m square. We accomplished this by commanding the robot to switch between driving forwards and turning left based on timing. Similarly to the Teleop Twist task, we published a twist using /cmd_vel Twist to make the robot move.  

This code was structured within a SquareNode() class, and switched between two states: “forward” and “turning”. In the forward state, the robot is sent a twist with a positive linear vector of (1.0, 0,0, 0.0) so that it moves forward.  Using this twist, the robot will drive forward at 1 m/s, so after 1 second has passed it will have driven 1 meter. After 1 second has passed in the driving forward state, the state switches to the turning state. Here, the robot is sent a twist with an angular vector of (0.0, 0.0, 0.5) which causes it to rotate left at a speed of 0.5 radians per second. After approximately pi seconds, a full 90 degree turn is completed, and the state is switched back to the forward state. This repeats until the robot has fully driven in a square.  

This was one of the first tasks we completed for this project, but later on, as we learned better practices for programming the Neato, we decided to go back and refactor this code. In the later version, there is less logic in the run loop. The current state is set to be a function corresponding to the current action. Once sufficient time has passed that the robot has completed an action, the function for the next action is returned, updating the current state. For example, we have a variable called curr_state, which is initially set to be equal to go_forward. Once the robot has moved forward far enough, curr_state is set to turn_left. Within a while loop, curr_state is set to be equal to curr_state(), so that whichever function the current state is equal to will run. We have found this to be a much cleaner, nicer structure, and it made it much easier to integrate our SquareNode() class with our finite state machine code later.

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

# Person Following

Our next task was to program the Neato so that it would identify a person in front of it and follow that person as they move around.  The first step of this challenge was to make it so that the Neato could recognize a person standing in front of it.

The neato’s lidar scanner that was used to control wall following behavior, was also used to identify the location of a person relative to the Neato’s base_link coordinate system.  In this behavior, as we designed it, the Neato will only follow a person if that person crosses or stands in front of it, within a certain threshold distance.  

We defined being “in front of” the Neato as being recognized within the 60 degree range directly in front of the robot, and less than a few meters away. To detect a person in this range, We we took the center of mass of the points corresponding to the measured distances from within this 60 degree range. When a person stands in this zone, there is a cluster of points detected by the lidar where the person is standing. The center of mass of this portion of lidar readings is located at this same position.   If that center of mass of the points corresponding to the ranges detected by the lidar scan in front of the robot is within the threshold distance, that position is identified as the location of the person, and is set as a target location. Otherwise, for example, if there is no person present or the person is too far away or behind the robot, the target location is set to (0,0). When the target location is (0,0), the robot will not move.

The graphics above demonstrate an example of potential lidar readings and how a person would be identified by taking the center of mass of the readings close to and in front  of the robot.

Once a person’s location is identified, that location becomes the robot’s target position. The robot’s angular speed is set to be proportional to the angle of the person’s position, and its linear speed is set to be proportional to the distance the person is from the robot. Once the robot has reached the person, the distance according to the lidar scanner will read approximately 0.75m, and the robot will stop moving forward unless the person moves again. The robot’s target location is being continually updated as it moves, so that when the person moves, the robot will change its velocity to continue moving towards the person.

For this task, the code was structured in an object oriented manner within a PersonNode() class. This class had a function to detect a person, a function to calculate the current velocity based on if there was a person detected and where, and a function to visualize where the robot believed a person was located. This visualization was very useful in debugging and in understanding how our instructions were being followed.

One problem we ran into early on, was that the Neato would sometimes detect inanimate objects as being a person and go to them instead. For now we improved this by setting a threshold to prevent the Neato from thinking that a wall across a room is the person to follow, but a way to improve this functionality in the future would be to convert positions to the odometry coordinate system and detect whether or not they were moving in this coordinate system to differentiate between people/targets and obstacles.

# Obstacle Avoidance

We approached the obstacle avoidance problem as a finite state problem. The neato moves between four states in this machine: move_towards_goal, turn_left, go_along, and turn_right. We initialize curr_state as move_towards_goal at the start of our run function. Each state corresponds with a function, and transitions between states are accomplished by returning a value that corresponds with the next function.

![obstacle avoider FSM](imgs/obstacle_fsm.png)

For clarity, let’s walk through an example:
As indicated above, curr_state is initialized to move_towards_goal. Within move_towards_goal, we make a call to go_forward to generate the appropriate twist. If the Neato detects an obstacle in front of it during the process_scan callback, then move_towards_goal returns turn_left and we transition to that state. Suppose we’re now turning left. The neato makes a 90 degree turn before transitioning to the state go_along. While the neato continues to detect an obstacle at its side, it “goes along”, moving away from the obstacle. Once the Neato has cleared the obstacle, the go_along function returns turn_right, and we make a 90 degree right turn. Upon completing that right turn, the neato transitions back into the state move_towards_goal. The path that the neato has completed at this point looks like the diagram at right.

![obstacle avoider](imgs/obstacle.png)

Given our time constraints, we made the decision that the Neato should always turn left to avoid obstacles. This means that even if another obstacle is immediately to the left, the Neato will head in that direction until it clears the first obstacle. Of course, this might result in the Neato crashing into the second obstacle. With more time, we would have programmed the Neato to turn either left or right in order to avoid obstacles, depending on which distance gives greater clearance.


# Finite state controller

Our finite state controller combined our drive square and person follower programs. The Neato starts off by driving in a square, repeatedly. As soon as a person is detected within the person following range, the Neato transitions to following that person. If the Neato loses sight of the person, it transitions back to driving in a square, as shown below.

## Drive square

As explained above in the [Drive square section](#drive-square), the drive square state, within itself, is made up of two states: going forward and turning left. Transitions between these states are time based: the forward velocity in the forward state is set such that 1 meter will have been traveled after 2 seconds, while the angular velocity of the turning state is set so that the Neato will have turned 90˚ after 3 seconds.

## Person following

As explained above in the [Person following section](#person-following), the person following state has the Neato drive, with forward and angular velocity proportional to the distance to the person, up to the person. A person is identified as the center of mass of the distance readings in front of the Neato from the lidar laser scan.

## Connecting the two

In our Python implementations of both drive_square.py and person_follow.py we used object-oriented programming, with all functionality necessary for the execution of a node split into methods belonging to that respective node's object. Thus, in order to gain access to all of this logic we had already implemented, we simply had to import both our ```SquareNode``` and ```PersonNode``` classes and instantiate them as class attributes in our ```FSM``` class of finite_state_controller.py.

Thankfully, since we originally implemented drive square as an FSM, every method of ```DriveSquare``` that represents a state returns the method that should be the next state: if the appropriate amount of time has passed (between the time of the beginning of this state and the current time) to signal the completion of the current state, reset the time for the beginning of the state and return the next state; else, return the same current state. This functionality allowed our main ```DriveSquare.run()``` method to simply initialize the current state as the forward state, and then, while the program has not been terminated, actually call the method of the current state (causing the Neato to do the desired behavior) and, afterwards, reassign the current state to be the return value of the method that was just called. There were only two such state methods: ```DriveSquare.go_forward()``` and ```DriveSquare.turn_left()```. This allowed for clean state transitions.

However, ```PersonNode``` was not implemented as an FSM, as there is only one action of person follower: if there is a person, follow it. This action is completed by the ```PersonNode.calculate_velocity()``` method, which determines what the Neato's velocity should be in order to successfully follow the person. In person_follow.py, this method was called over and over again in a while loop of the main ```PersonNode.run()``` method. So, in order to make it a state, it just had to return itself! Adding this simple edit to person_follow.py did not affect our original person follower program.

Now that we had access to all of the needed states, we had to be able to transition between driving in a square and following a person. In the main ```FSM.run()``` method, we create a current state ```curr_state``` variable and instantiate it to ```self.drive_square.go_forward```, making the start state of the FSM the go forward state of the encompassing square state. Then, within a while loop, we check if a person has been identified, and, if so, reset ```curr_state``` to be ```self.person_follower.calculate_velocity```. Else, if ```curr_state``` is equal to ```self.person_follower.calculate_velocity```, then the Neato had been in the person follower state but just lost sight of the person. As a result, we set ```curr_state``` back to ```self.drive_square.go_forward```, the beginning of the overall square state. If neither of these two conditions are true, that means that the Neato was previously driving in a square and should continue to do so, so we do not reset ```curr_state```, as the square state should transition between its two internal states on its own. We then actually call ```curr_state()``` to allow the Neato to execute the method save to ```curr_state```. ```curr_state``` is then set to the return value of this method call, so that everything goes smoothly in the next run of the while loop.

We check if there is a person before executing ```curr_state()``` in order to allow the Neato to start following a person immediately. If we called ```curr_state()``` before checking if there was a person, the Neato would continue its previous action before actually going towards the person.

### Process scan callback

Being able to execute the current state and set the next state is great, but is pointless if there is no implementation for triggering the transitions between the drive square and person following states. Thus, we subscribe to the ```LaserScan /scan``` topic and have a callback ```FSM.process_scan(m)```. Within this callback, we call ```self.person_follower.process_scan(m)``` and ```self.person_follower.find_person()```. Calling these two methods allows the ```PersonNode``` logic of identifying a person based off of laser readings to be executed every time there is new laser data. This allows person identifying to be independent of the actual execution of the states in ```FSM.run()```.

### Publishing Twists

All three of the states are executed by the Neato by publishing a Twist velocity over the ```/cmd_vel``` topic. In order to keep this publishing within ```FSM``` and not in the ```SquareNode``` or ```PersonNode``` classes, every state method sets a ```self.twist``` attribute to the appropriate Twist, where ```self``` is the appropriate class of that method. Then, in the while loop of ```FSM.run()```, in the same places where we set ```curr_state``` to the appropriate person following or drive square methods, we also set ```self.twist``` of our ```FSM``` object to be the ```twist``` attribute of the appropriate class. For example, if there is a person, we set ```self.twist = self.person_follower.twist```.

Then, after ```curr_state()``` has been executed, we publish this ```twist``` attribute of ```FSM```.

## Difficulties

One of the main problems we had was figuring out how to include our person follower logic, which was not originally implemented as an FSM, into our finite state controller. After realizing that our ```PersonNode``` had one main action for the Neato to execute (move in such a way that follows the identified person), we were easily able to modify our person follower code to be usable by our finite state controller.

We also realized how important it was for our code to be modular. Every method should only do one thing. The


# Conclusion

Over the course of the last few weeks, this project has been a great learning experience for our group. We have become much more familiar with using ROS and how to program robots. We have learned better practices for writing and structuring our code. We began with simply subscribing to and publishing topics. When we moved on to an object oriented structure it became much clearer to us how we might better separate the logic into different functions, and we were better able to approach making the robots follow more complex behaviors. After this we got more practice with controlling the robot using a finite state machine. This type of control was both intuitive to understand and effective to achieve the desired behavior of our Neato.  

We did run into some challenges along the way throughout this project.  Things did not always behave the way we expected them to. We learned that using timing to control position after movement is never going to be perfect, and in the challenges where we were detecting obstacles, walls, and people, it was not easy to figure out how we should interpret data from the lidar scan. Our experience working with the robot’s sensors taught us that there are often going to be situations where we are given data that is not useful, and we need to know how to handle those situations when they arise.

If we were to have more time to work on this project, there are a few things we would improve. We would put more time into making our code even more readable and clearly structured. There are also a couple clear points we identified that could improve functionality. Detecting when obstacles are moving in the odometry frame would allow us to identify people more accurately, and would lead to fewer instances of obstacles being misidentified as people, and using the odometry to sense position to control movement would be more accurate than using timing.

Though there are still areas with potential for improvement, we believe this project was a success. Accomplishing these tasks controlling the neato was at times challenging, but we were able to figure out solutions and learned a lot in the process.
