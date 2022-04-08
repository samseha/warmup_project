# warmup_project
Driving in a Square

A high-level description: The problem is to make the robot drive in a square.

Solution: I used the sleep function to make the robot go straight and turn 90 degrees after a certain time.

Code:
__init__: Initialize the node and a publisher for /cmd_vel. Also create an empty Twist(). 
Gave 1 second to sleep so that there is enough time to set up.
square: sets the initial x velocity to 0.1 and publish the velocity. Then sleep after 10 seconds.
Publish 90 degrees and sleep 1 second and repeat.

Video:



https://user-images.githubusercontent.com/57845592/161667696-eea22f37-ce0b-4154-b212-0d4d17d4c9c7.mp4



Person Follower

A high-level description: The problem is to make the robot follow a person in its direction and stop at a safe distance facing the person. I took the LIDAR scanning and found the closest distance and its angle so that the robot would move towards that direction.

Solution: I took the LIDAR scannings and if there were no objects nearby the robot would not move. If there were objects nearby I would take the minimum distance and its angle so that it would move towards that direction with a fixed speed. Here I ignored angles less than 10 degrees as it would make the robot behave weirdly when its too close to the person. If the robot reaches the set distance of 0.5 it would stop.

Code:
__init__: Initialize the node and a publisher for /cmd_vel and a subscriber for /scan with a callback function. Also create a empty Twist() to save the velocity of robot.

best_angle: Takes in an angle and returns 0 if less than 10. If larger than 180 returns angle -360 to prevent turns larger than 180. Else it would return original value.

process_scan: Takes in the scan data and finds the object with minimum distance. It then publishes appropriate angle, velocity data to the robot depending on distance and angle.

run: keeps the program alive.

Video:



https://user-images.githubusercontent.com/57845592/162511138-d6b69a70-a578-42ce-98b5-eaae213a6c6e.mp4



Wall Follower:

A high-level desription: The problem is to make the robot find a wall and follow the wall turning at corners appropriately. I used distances at different angles of the robot to calibrate it.

Solution: I used the distance from the left side of the robot to find if the distance at 45 and 135 degrees were different and used that to make adjustments. In addition, I used the distance between the wall and the safe distance such that it would make turns appropriately. Since the balance angle will be 0 when travelling along a wall, as the robot gets close to the front wall it will start turning. For velocity I used the k*e formula we learned at class. However, instead of the error term I just used the distance from the front wall as the robot needed speed to turn at corners. Also for values such as distance I tested out different values but realized the robot needed a buffer to succesfully turn without bumping the wall.


Code:
__init__: Initialize the node and a publisher for /cmd_vel and a subscriber for /scan with a callback function. Also create a empty Twist() to save the velocity of robot. Also create a Boolean called at_wall to see if the robot is near a wall
get_x: gets the x velocity for the robot. First check if robot is near a wall by going through all lidar readings. Speed will be proportional to the front wall and if it is larger than 0.3 we will limit that so that the robot does not go out of control.

get_angle: If the robot is not near any wall we return 0. If not, we get the distance from different angles at 45, 90, 135 degrees. We calibrate while at wall with the distance between 45 and 135 degrees. And for turning we use the difference between our "safe distance" and the distance from 90 degrees.
We also check for cases where there are no readings and respond appropriately.

process_scan: Get velocity and angular data and publish it. Also checks if robot is near wall every time we get twist data.

run: Keeps the program alive

Video:



https://user-images.githubusercontent.com/57845592/162511156-8e0773fa-7acd-4ee7-ba74-3f2ef9828695.mp4




Challenges: The biggest challenge is that there are some discrepencies between Gazebo and a real turtlebot. For example, I do not think there are 0.0 readings in Gazebo, so you have to make sure the code accounts for all cases. Also, there are bumps in CSIL5 in the middle of the room for power outlets. If the robot is slow it will not drive over that bump which causes it to get stuck. Also, initially my approach was to get exact angular turn values at a certain time. However, it is hard to expect the robot to behave 100% perfectly unlike code where you are solving a simulated problem. In the wall_follower code I had to scrap all of my code and use a more calibrating approach. I believe that this approach is better since it is hard to assume that robots will turn the exact angle and due to residue speed it will not travel the exact distance you want it to. For future projects it will be important to take account of this as problems will become more complex.


Future work: If I had more time I would try to fix the issue I had with my original approach for wall_follower. I do believe it is still possible to find a way to implemnt the exact approach using 

Takeaways (at least 2 bullet points with a few sentences per bullet point): 
- It is harder to find exact measurements and to implement them on robots since they sometimes behave differently.
Due to friction, momentum, etc, robots will go more than planned and its better to calibrate then instruct for exact movements.

- It is important to have generous buffers for measurements.
Similar to the point above it is important to buffer for limits. Since robots do not behave exactly, we need to account for errors which can get large if they accumulate.
