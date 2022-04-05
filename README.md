# warmup_project
Driving in a Square

A high-level description: The problem is to make the robot drive in a square.
Solution: I used the sleep function to make the robot go straight and turn 90 degrees after a certain time.

Code:
__init__: Initialize the node and a publisher for /cmd_vel. Also create an empty Twist(). 
Gave 1 second to sleep so that there is enough time to set up.
square: sets the initial x velocity to 0.1 and publish the velocity. Then sleep after 10 seconds.
Publish 90 degrees and sleep 1 second and repeat.

Gif:



https://user-images.githubusercontent.com/57845592/161667696-eea22f37-ce0b-4154-b212-0d4d17d4c9c7.mp4



Person Follower

Problem:
Solution:

Code:
