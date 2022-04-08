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


Wall Follower:
