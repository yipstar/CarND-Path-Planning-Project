The following writeup will describe how my project meets each individual rubric point, as well as providing a description of the design and implementation of my path planner.

# Rubric Points

### The car is able to drive at least 4.32 miles without incident..
On most simulator runs my code is able to make it around the track without incident, usually to around 4.8 miles. Things break down there as I think sensor fusion stops working.

### The car drives according to the speed limit.
I programmed the system to use a spped limit of 25.5872 m /s, which is 57.23 mph. I then defined a 4 m/s buffer to arrive at a target speed, of 21.5872 m/s which is about 48.2 mph. You can see these constants defined in (./src/AbstractTrajectory.h lines 38-40). All trajectory generated at the trajectory planner layer level respect this target speed, meaning this is the maximum velocity boundary condition that any generated trajectories attempt to use.

### Max Acceleration and Jerk are not Exceeded.
All trajectories generated from the trajectory planner layer use Jerk Minimizing Trajectories as described and implemented in the Trajectory Generation lessons. When generating trajectories, my trajectory generator will create a set of jerk minimizing trajectories, and then filter out any trajectories that exceed the maximum acceleration. You can see this code in the AbstractTrajectory::filter_trajectory_set function (./src/AbstractTrajectory.cpp beginning line 162). In addition to removing any generated trajectories in the trajectory set for trajectories that exceed the maximum allowed acceleration, I also filter trajectories that exceed the defined target speed, travel past the inside and outside lane boundaries, and any trajectories that are protected to lead to a collision.

### Car does not have collisions.
In both my behavior layer and trajectory generation layer I check all generated trajectories for collisions with other cars, and penalize these trajectories using cost functions. For the behavior planner, you can see this code in the BehaviorPlanner::collision_cost function (./src/BehaviorPlanner.cpp starting line 230). For the trajectory generator, you can see this code in the AbstractTrajectory::check_for_collisions function (./src/AbstractTrajectory.cpp starting line 249).

### The car stays in its lane, except for the time between changing lanes.
All trajectories are generated using FreeNet coordinates, and never change the d value unless when changing lanes. All lane change trajectories use a 3 second jerk minimizing trajectory that keeps velocity constant. You can see this code in the ChangeLaneTrajectory::make_trajectory function (./src/ChangeLaneTrajectory.cpp starting line 47).

### The car is able to change lanes.
When in the 'KeepLane' state the behavior planner will use a cost function that checks to see if a car is in front of it and driving below the speed limit and will attempt to change lanes as long as there is not a car in the target lane that is too close to it. The behavior planner will also check the lane change trajectory for collisions. You can see this code in the BehaviorPlanner::calculate_cost function (./src/BehaviorPlanner.cpp starting line 126).

### Reflection
I tried to design the path planner using the architecture that was described in the lessons. I keep track of how many points were traveled in each simulator timestep and use that as a system timer (num_points_traveled * .02 = dt). The system has 3 main modules, a Behavior Planner, Trajectory Generator, and Prediction module.

The Behavior Planner is realized in the BehaviorPlanner class and is setup to run every 250 points (5 seconds). It is coded as a finite state machine with 3 states 'KeepLane', 'LaneChangeLeft', and 'LaneChangeRight'. It uses cost functions to implement state transitions, and returns a Maneuver structafter calling its update_state method. The Maneuver struct was meant to model the output data structure described in the Behavior Planning lesson, however its only real purpose in this system is to indicate the target lane when transitioning to a change lane state. I had hoped to model a more sophisticated behavior layer however I spent too much time figuring out trajectory generation and how to interact with the simulator properly, so this is an area of the project I would like to further explore in the future.

The Prediction layer is realized in the Prediction class and it runs every time my program receives an update from the simulator. It is very basic in that it simply generates a 5 second time horizon trajectory for each car returned from sensor fusion, using a simple constant velocity trajectory.

The Trajectory Generation layer is realized in several concrete Trajectory classes (KeepVelocityTrajectory, ChangeLaneTrajectory, GetToTargetSpeedTrajectory, and ConstantVelocityTrajectory). There is also a main class TrajectoryGenerator that was meant to hide the details of these different classes behind a simple interface but due to time constraints I didn't clean this entire layer up as well as I would have liked. Let's just say I am in no way proud of the code design in terms of polish of any of this project. I look forward to getting the chance to rewrite the project in its entirety from scratch with the lessons I have learned from my first attempt.

While I coded several different concrete Trajectory classes they all use a common base class AbstractTrajectory. This was an attempt to use a template method pattern to reuse a common structure and allow different trajectory strategies to vary it certain ways, but it's clearly half baked.

The GetToTargetSpeedTrajectory is just used to generate an initial ramp up trajectory to get the car towards the target speed from its initial starting position, it's probably not necessary for this to be a specific type in its own right but it got the job done so I left it in place.

During every cycle returned from the simulator lots of bookeeping is performed and then either the KeepVelocityTrajectory is invoked when the Behavior layer is in the KeepLane state, or the ChangeLaneTrajectory is invonked when the Behavior layer is in either the LaneChangeLeft or LaneChangeRight state.

Both the KeepVelocityTrajectory and ChangeLaneTrajectory use the next 10 points from the previously generated path to ensure continuity of subsequent path generation cycles. They also both use the provided JMT function (./src/Helpers.cpp) to create jerk minimizing trajectories.

The ChangeLaneTrajectory only generates a single target trajectory that fullfills a 3 second jerk minimized lane change with constant velocity.

The KeepVelocityTrajectory however generates a set of jerk minimized trajectories in the KeepVelocityTrajectory::generate_trajectory_set function (./src/KeepVelocityTrajectory.cpp line 14) that vary in the final target velocity, and then filters out trajectories that not feasible or comfortable, and uses a simple cost function to rank them. Thus when the BehaviorPlanner is in the KeepLane state, the KeepVelocityTrajectory (inappropriately named, again not proud of this code due to time constraints) chooses a trajectory that penalizes cost when it gets close to cars ahead of it using a simple cost function.

The cycle of Behavior running every 5 seconds, and Prediction and Trajectory Generation running every update from the simulator encompasses the system design of my overall path planner.
