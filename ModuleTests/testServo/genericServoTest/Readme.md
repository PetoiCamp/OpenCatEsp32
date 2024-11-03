# Introduction
The robot has to be calibrated, and utilize the full OpenCat library to walk well. This simple code will only generate walking-like patterns but will not effectively move forward. It can be a starting point that you learn a walking system.

# To upload the code
NyBoard: Move the folder src/PetoiESP32Servo to temp/, then select NyBoard in Arduino IDE and upload. 
BiBoard: Move the folder PetoiESP32Servo back to src/, then select BiBoard in Arduino IDE and upload. 

# To demo
Enter the following letter in the serial monitor
* 'c': move the joints to calibration posture
* 'd': move the joints to rest posture, and turn off all the servos
* 't': move the joints to stand posture
* 'a': rotate all the servos around the zero position
* 'k': make the robot walk (simple walk)
* "k offset amplitude":  tune the parameters of the simple walking in the serial monitor.

For example, k 5 8 will cause the robot to lean forward by 5 degrees and increase the step amplitude to 8 degrees. 
