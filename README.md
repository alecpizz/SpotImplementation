Credit to: https://github.com/MASKOR/webots_ros2_spot

Setup:
Launch the world in Webots.
Edit spot_controller.py to extend the project.
To modify the maze, use the MazeGen proto node in the scene tree. The are fields for defining start end exit points, alongside the maze itself. The maze is represented as a string of 0's and 1's like this:
1111111100100001101010111000000110101011101010011100000110101101

Which is interpreted as:
```
11111111
00100001
10101011
10000001
10101011
10101001
11000001
10101101
```

Where 0s are avalible paths and 1s are closed off. 

See FOR NEXT CAPSTONE.md for more information as well as REPORT.pdf for a full writeup.