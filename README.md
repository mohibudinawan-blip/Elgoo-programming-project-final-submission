PROJECT OVERVIEW:
Code base for ELEGOO R3 UNO robot (by Mohsen Mirza and Mohib Awan), coded with C++ in Arduino IDE.

STRUCTURE:
-CPP_Exam_Code (Code for the final exam)
-Past Projects (individual mini projects built during given lecture time)
-Functions (individual sects of code with delibrate functiontionality, packaged for easy reuse)
-Project Development (main project folder where projects are developed, a basic essentials file by Eric Rapos (contains motor pins, servo and gyro base functions, etc...) is available, as well as a project file containing a basic project structure for easy development. 

HARDWARE: 
The R3 comes with a set of motors (for wheels), servos, and two sensors (gyro and ultrasonic).
HARDWARE ISSUES:
-Wheels have a tendancy to shift from place after repated use, causing the movement of the robot to become slanted. Long term fix is to tighten the screws of the motors as well as coding in a counter to the direction it's drifting in. Quick method would be to manually move the wheels by hand into the correct position. Re
-ultrasonic sensor has issues with consistently reading the correct light levels. For example the sensor is placed on an area that has a light level of 50, it will momentarily output 300 even though the reading should be 50. A fix for this is creating a special case where the robot discards the outlier reading so it behaves as expected.

INSTRUCTIONS:
-Open a terminal (Git Bash, PowerShell) and type "git clone https://github.com/mohibudinawan-blip/Elgoo-programming-project-final-submission.git". This will clone the git repository.
-Open Arduino IDE and connect the R3 with your device via blue cable.
-Make sure you have the corrects boards installed, it should be labeled "Arduino UNO".
-Once connected, to upload your code click the arrow at the top left.
-To run the R3 click the cylindric button on its side. To reset the robot click the tan colored button on top (same area has the port).


