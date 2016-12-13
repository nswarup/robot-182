# robot-182
Code for self-balancing robot using Q-learning with a Fourier Basis. 

pidbot.ino and pidlearn.py are the files that allow the robot to seed its value approximation function with the baseline PID algorithm, and then UprightRover-V3-1.ino and robotinit.py allow the robot to continue learning to balance afterwards. Unfortunately, the read/write serial communication to the Arduino is too slow for this computer-Arduino approach to work, but in the future we will directly move the python code onto an Arduino board with higher processing power.
