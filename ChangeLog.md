# Change Log

## Apr 11, 2022
* Optimized the power saver logics: random mind after idle, then go to rest.
* Avoid servo jumps caused by shutServo()

## Mar 15, 2022
* Mirror the leftward gaits for rightward gaits to save space.
* Update token system: i->I, M->i, l->L

## Mar 10, 2022
* Add QA routine for EEPROM, IMU, and IR.
* Add token 'R' to soft-reboot and enter reset routine. 

## Mar 1, 2022
* Support IOS Ble app.
* Correct the angle range of m token (use int instead of int8_t).

## Feb 26, 2022 
* Correct an error when reading commands. The remaining serial commands should be checked within the token's available condition. 

## Feb 22, 2022
* Change skillList to a pointer and init it in setup().
* Correct the lastCmd/newCmd logic of the initial posture.

## Feb 19, 2022
* Add macro for GYRO.
* Make the robot enter the joint calibration state (different from initialization) if it is upside down. Otherwise, start up normally.

## Feb 8, 2022
* Reduce the odds of random noise when shutDown servos by detaching them then re-attach. 
* Improve the sound for falling over.

## Feb 7, 2022
* Set the bootup posture to calibrate state, for convenient installation. Disable gyro and auto behavior until first user input. 
* Optimize servo angle range.
* Merge the token FOLD with regular skills.
* Tune the pee, hi, and balance skills.
* Optimize low battery checking logic. Safely transform to rest posture then shut the servos. 
* Tune LED.
* Tune adjustment damper.
* Remove the delay between postures.
* Add special sound for falling over.


## Feb 6, 2022
* Add a 'v' token to print the MPU for once and a 'V' token to print the MPU for every loop. 
* Fix a bug when reading IMU acceleration data.
* Add comments the difference between REAL_ACCELERATION and WORLD_ACCELERATION.
* Correct the servos' joint angle range.
* Add a low voltage alarm for BiBoard2.
* Make NeoPixel blink with motion for BiBoard2.

## Feb 5, 2022
* Add T_PRINT_GYRO 'V' to toggle printing of Gyro data. 

## Feb 2, 2022
* Merge PCA9685 PWM driver in the main code to be activated with proper macro.
* BiBoard2 works!

## Feb 1, 2022
* Implement writeAngle() outside the Petoi_PWMServoDriver Class.

## Jan 29, 2022
* Fix the infinite loop of "play dead - recover" and "m (or other non-skill token) - recover".
* Use hardware PWM tone library to generate beep and improved the music quality.

## Jan 27, 2022
* Create a token 'z' to turn on/off the power saver and random behaviors.
* Create a token 'r' to change the adaption directions on a ramp.
* Define fold and rotate behavior for cub.
* Add I2C detector and macro for IR.

## Jan 26, 2022
* Write the log up-to-date.
* Restructure T_SKILL reaction to simplify the control flow.
* Fix bug in power saver.
* Improve power saver and randomMind logics.
* Fix adjustment jump in recovery routine.
* Robot resume previous status after recovery.
* Study ESP32 timer for PWM. Find the limitation for servo frequency.

## Jan 25, 2022
* Add a random behavior generator when the robot is idle. The list can be predefined and support all kinds of tokens. 
* Fixed the bug of allocating large skill data array by redesigning the Skill class.
* Add a new melody for the initialization stage.  

## Jan 24, 2022
* Add a power saver to bring the robot to rest after being idle for 15 seconds.
* Try to locate the bug with a large memory operation for  T_SKILL_DATA.
* Add a new token, T_SKILL_DATA, to receive the skill data from the serial port in real-time and perform the skill locally to avoid the overhead of communication or slow compiling and uploading in Arduino IDE. Compatible with all three types of skills.

## Jan 23, 2022
* Support the 12 DOF model. The code works on Nybble, Bittle, and the 12 DOF model by changing a single macro definition.
* Enum Petoi models of servos and updated the initializer for servos.
* Automatically set the Bluetooth device name for the three models.

## Jan 22, 2022
* Generate a finer gait for Nybble, and adjust the code for Nybble with additional head and tail joints. 
* Fix the jig when changing from gait to posture by resetting the frame number.
* Add a damper after a long behavior to reduce the jump in the balancing adjustment value.
* Make the head and tail joint work to support Nybble.

## Jan 21, 2022
* Add a macro to go through the initialization stage automatically.
* Add a tag to fix the problem when Bluetooth instructions are skipped in the main program due to multi-processor synchronization. 
* Add multiple serial printing methods to support the Bluetooth app.
* Generate the Bluetooth device name with two random Hex numbers at initialization and save it to EEPROM. The name won't change unless the board's "birthmark" is changed. 
* Improve the EEPROM I/O functions.
* Fix the bug with 'p' and 'g' tokens.

## Jan 19, 2022
* Add ESP32config instructions to fix the MPU6050 lock. 

## Jan 18, 2022
* Write I/O for the App in the main code.
* BLE connects to the App.
* Write a skill to climb up a steel table with magnets.

## Jan 16, 2022
* Add 'p' and 'g' tokens.

## Jan 14, 2022
* Write a generic routine to fix the bug with the large String buffer for 'i' and 'l' tokens.
* Fix the indexing bug when changing walking DOF on the 12 PWM board.

## Jan 13, 2022
* Write the macro to automatically decide WALING_DOF and DOF with different combinations of models and boards.
* Fail to run the MP3 demo due to the upgrade from ESP 1.0 to 2.0.
* Tune the char string style of cmd.
* Tune motion adjustment parameters.

## Jan 12, 2022
* Structure multiple modules' libraries for the whole project.

## Jan 11, 2022
* **Adjust the servo library to fix the PWM channel conflict!**
* Locate the cause of detached servos
* Add a mirror function to flip the left/right side of movements randomly. 

## Jan 10, 2022
* Investigate the relationship between the PWM channel and detached servos.

## Jan 9, 2022
* Move dependent libraries into the project folder to help new users.
* Add calibration routine for MPU6050 and save the offsets in EEPROM during initialization. 
* Add a "birthmark" to know if a board has been initialized so that it will boot up directly next time.

## Jan 8, 2022
* Solve the detached servo problem by re-attaching the corresponding channels to continue other parts. 
* Joint calibration routine works. 
* MPU works but without calibration. 

## Jan 1, 2022
* Fix the bug when switching to behaviors by correcting the servo indexes on a 12 PWM board.

## Dec 30, 2022
* Behavior partially works.
* Add a boolean to check if IMU is updated and decide if to recalculate the balancing adjustment values.

## Dec. 28, 2022
* Servos can move.

## Dec 16. 2022
* Improve the code and demo's doc for serialMaster.

## Dec. 10, 2022
* Fix the esp32-hal-i2c-slave.c bug that locks IMU at booting stage.
 
