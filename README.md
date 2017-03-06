# Robotic-Control-via-Vision
STATEMENT OF THE PROBLEM

The objective of this project is to construct a mobile robot that is able to build a structure which consists of Lego pieces based on real-time feedback from a webcam. The mission is to be completed within the guaranteed time duration of 25 minutes. The workspace consists of a white 3x4 ft2 board which serves as the background of the image. Lego pieces of various colors and sizes will be randomly distributed on half of this workspace, as illustrated in Figure 1. The robot is to be placed in a random location at the starting region. The robot is to identify and localize Lego pieces with use of the webcam which provides imaging feedback. The robot must construct the three alphabet-shaped structures with Lego pieces in the location specified in Figure 1.

￼
Figure 1 - Workspace layout

The two Lego dimensions available are small 1.6cm(length) x 1.6cm(width) x 0.96cm(height) and large 3.2cm(length) x 1.6cm(width) x 0.96cm(height). The three colors used for this project are blue, red and green. The structures the robot must construct are the letters U, H, and I in the Lego configuration illustrated in Figure 2. The letters must be aligned horizontally in a straight line. 
￼
Figure 2 - Configuration of Lego pieces for letters

METHODOLOGY

After thorough research on approaches in computer vision, the use of OpenCV library of programming functions for real-time image processing was selected. This selection was based on a number of factors. Although sample code was provided to all the teams in room H-1066, the lack of available information on the system used as well as the overcrowding of students on the four available computers greatly influenced the decision of using OpenCV. Additional factors include the amount of online resources available for OpenCV and the short period time. 

The objective of this project is to use computer vision to allow the wheeled mobile robot to accomplish a set the tasks which includes: 
identifying the location, colors and size of all the Lego pieces;
find the closest block of a certain color;
move towards the block;
pick up the block;
move towards the workspace area; and
Place the block in a systematic order such that the structure the blocks create yields the letters “UHI”.
These systematic steps create a control system which is event-based scheduling as oppose to time-based scheduling. The reason for this is because each event can only occur once the preceding event has been completed. In other words, each tasks occurs not at a specific time but after the completion of a specific prior event. 

DESIGN
Hardware
A list of the specific components with their respective prices is found in Appendix A. 

Aluminum Shield
An aluminum shield covers this wheeled mobile robot. The purpose for this is to attach a white paper with two yellow dots of different areas on top of the robot. These yellow circles enable the robot orientation to be identified by the visual program, which will be described later. The small yellow circle is located on the center of rotation of robot to reduce the amount of adjustments needed when the computer vision program instructs the Arduino to rotate clockwise or counter clockwise.

Wheels
Illustrated in Figure 3 is an assembled depiction of the constructed wheeled mobile robot. There are two wheels at the front of the robot, each controlled by separate servo motors. These serve to control the direction and position of the robot. The back wheel is a ball caster to allow for a turning circle based around the two front wheels. Additionally there are two servo motors: one to control the height of the clamp and the other to open and close the clamp.  The base of the robot has been 3D printed from a one off design made to incorporate all the features required to mount the components and keep its footprint to a minimum. This includes grooves for the servo motors, holes for mounting points, tabs to space the circuit board for mounting and a housing for the battery back at the back of the robot to counteract the weight of the servo motors.


￼
Figure 3 - Solidworks model of wheeled mobile robot

An Arduino has been incorporated into the design along with a Bluetooth shield. For the Arduino to receive commands from the main program: the computer outputs its signal through Bluetooth as it would through a serial port, this information is received at the Bluetooth shield and operates the servo’s as desired. Bluetooth was incorporated to reduce additional forces and friction caused by an attached cable. Using a cable could restrict the movement of the vehicle, accidently hit blocks and confuse the visual system. The Bluetooth system is initially more work to setup and program however it proves beneficial for testing and reducing movement restrictions.

Power Source
Because the system is wireless the robot requires its own internal power source to operate, as previously mentioned a battery is incorporated in the design. The battery chosen to match the Arduino is a 7.4V battery, however the servo motors are suggested to run between 4.8-6V, therefore a small 5V voltage regulator was implemented to supply power to the servo’s. 
Wheel Blocker 
Due to the type of driving wheels and their high level of traction, when confronted with a Lego block the robot had a tendency to climb over or flip surrounding blocks when driving around. To solve this problem a wheel blocker was installed that pushes blocks that are in the way to prevent them from being run over.

 Gripper Modification
Initially the scope chosen for of our project only included the use of small Lego blocks due to the inability of the unmodified claw to grab the larger blocks reliably. To account for this problem later in the project a gripper extension was made, this acts as a funnel for larger blocks and re-orients them to fit in the claw.

Software
The nature of the project concerns event-based scheduling. Thus the order of the tasks and their completion is of importance from a programming standpoint. The complete program can be found in Appendices B and C for the main.cpp and Arduino.ino codes, respectively. Due to the linearity of the system’s behavior, no additional controls, such as PID, are required. Moreover, the light design of the wheeled mobile robot allows for the robot to climb over the Legos without moving them significantly. The ability of the claw to be raised and lowered aids in the avoidance of moving the undesired blocks significantly. This allowed for the program to be simpler in terms of omitting functions responsible for obstacle avoidance and path finding.
Bluetooth 
The program has been organized into two main classes. The bluetooth class mainly just organizes what is required for serial communication between the main program and the Bluetooth shield which is attached to the Arduino microcontroller. The member functions associated with this class include a constructor and destructor—where the constructor allocates a location in windows memory to use as the serial port transfer, and the destructor closes the file. During the operation of the system, it was found necessary to close and open the serial port file, as it was noticed there was occasional loss of communication with the Bluetooth hardware which required physical intervention. Therefore after ever 5 blocks that have been moved, the Bluetooth closes and re-establishes communication. 

Image processing
The lego_collection class possesses all the variable and member functions required for image processing and event-based scheduling. Its constructor initializes the variables of the program as well as contains an array of coordinates for the letters UHI. The member function of class lego_collection represent the steps necessary to complete the objective of the project. 

The Image_Filtering() member function filters the original image into three separate threshold variables in accordance to the colors red, blue, green and yellow. This is achieved by first blurring the image via OpenCV’s function blur() in order to reduce possible noise. Next, the image color scale is changed from red-blue-green (RBG) to hue-saturation-value (HSV) through the use of the cvtColor() function. The HSV image facilitates the process of isolating the different colors whereby each color’s range of HSV is defined in the inRange() function. The approach of locating the blocks based on color instead of shape is to eliminate the noise caused by shadows when attempting to search for rectangles. The characteristic of this project allows this approach to hold true since the lego pieces are all rectangular thus eliminating the requirement of filtering shapes. 

The Block_locations(Mat MAT_threshold, char debug_name[]) member function searches for blocks of a given color and outputs their location. This is achieved by first inputting the desired color into the function. Next, with use of the threshold image of a given color, the OpenCV function findContours() creates counters around all objects in this image. This creates moments and center of mass vectors to all the objects contoured. For each object contoured, the area, defined by OpenCV as a pointer of the contour’s moment Moment.m00, of the object must be above a certain value to eliminate possible noise. The coordinates of the blocks above this noise threshold are then calculated by dividing the x and y moments by the area of each object. The coordinates are then stored in an array for further analysis in other functions.

The Robot_location(Mat MAT_threshold, char debug_name[]) member function receives data concerning the location of the two yellow circles which represent the front and back portions of the robot, respectively. These two circles are differentiated in the program by their areas. Once established which circle represents the front and the back of the robot, their coordinates are calculated and saved in memory for further analysis. 

Move robot
In order for the robot to spell out the desired UHI two arrays were created to store the coordinate and color information. Dropoff_array is a 3x27 array which has 1 row of information for each block to be picked up. For each row 3 columns contain the desired color, desired size and block number. The block number is then used in coord_array which is a 2x27 array which contains the x and y coordinates for each dropoff location. The coordinates are organized in such a fashion that the blocks will be dropped off to spell UHI with the desired spacing. As the code iterates through the move robot function it increases the block_number after each dropoff coordinate is reached. Once the block_number is increased the whole process is restarted.

Given the completion of these member functions, the distance between the robot and all the blocks of a specific color are calculated in the Closest_block_distance(vector<Point2f> Block_Centroid) member function with the use of the Pythagorean theorem. The function only occurs once before the robot begins moving. This approach reduces the amount of time required for the program to be executed since the terrain will only need to update the location of the robot throughout the steps until the next block needs to be identified and located.

The member function update_image() updates the image for the yellow threshold image in order to relocate the wheeled mobile robot. Moreover, the member function Update_block_distance(Point Block_Centroid) recalculates the distance between the robot and the closest block. This is necessary to capture the occurrence of when to stop the advancement of the mobile robot and prepare it to pick up the block.

The Closest_block_angle() member function calculates the angle using arctangent between the closest block and the robot’s two yellow circles. This is essential in rotating the robot to ensure proper alignment between the robot and the block. If this is not achieved, then the robot would miss the block. Similarly, the Angle_to_DROPZONE(float xcord, float ycord) member function concerns orienting the robot with the coordinate whereby the robot will release the block in order to create the letters UHI.
The order and conditions of all the previously mentioned member functions is set in the Move_robot(int block_location) member function. Firstly the Robot_location and the Update_block_distance are called in order to establish the current location of the robot in relation to the closest block. 
The status of the claw is then considered. If the claw is open, the robot is programmed to attain a block within its grasp. To do this, the program locates the closest block to the robot and sends a command to the Arduino that orients and moves the robot towards this block. When the robot is within a certain distance of the block, the program sends a command to the Arduino to stop the robot from advancing and then to orient the robot’s claw within one degree of the block. This increases the precision of the robot to pick up the block when programmed to do so. Once the claw is closed, the program assumes that the robot is holding a block. The robot orients and moves towards a particular coordinate in the workspace region until it reaches a certain distance from the desired coordinate. Here, the robot orients itself to be within one degree of the coordinate the block is required to be located. Once the robot has released the block at the desired location, the claws are open which thus begins moving the robot to the next desired color which would then be placed at the next pre-established coordinate.

Main Loop Procedure
The main() function activates the webcam. The serial communication sends the command outputted by the Move_robot member function to the Arduino via bluetooth. The Arduino has been preprogrammed to behave in certain ways in accordance to the serial input. For example, when the Move_robot member function outputs the command 'w', the serial communication bring this char to the microcontroller which possesses a number of cases, one of which is case ‘w’. The programmed commands available to the Arduino are found in Table 1.  Under this case, the microcontroller is programmed to turn both wheel’s servo motors forward for a short period of time. Therefore, the communication of the image processing program main.cpp and the servo motor control program Arduino.ino allow for the desired overall behavior of the robot to complete the required tasks.

Table 1– Arduino commands
Command	Function
w	forward
a	left
s	reverse
d	right
t	forward fine
f	left fine
h	reverse fine
y	right fine
i	forward cont.
j	left cont.
k	reverse cont.
l	right cont.
g	grab
r	release
v	arm up
b	arm down
n	claw open
m	claw close
q	detach all


Arduino Code

During operation the Arduino is continuously listening for commands from the serial port. Once a command is received that matches a command stored within the Arduino program, that command is then executed using its own delay and pin out to the appropriate servo motor.

ANALYSIS
 Considering that the width of the claw is approximately the same size at the width of the large Lego, this imposed a design constraint on the system. To resolve this issues, angled clamps were incorporated into the design of the robot. In addition, the robot was programmed to move left and right to move the Lego in order to grasp the Lego horizontally. This increased the efficiency of picking up the blocks to 90% calculated from experimental trials. The 10% error is caused when the scattered blocks are too close and the claw descends onto this close block. This misses to pick up the desired block thus leading to the placement of an inexistent block at the specified coordinate. Due to the three month time constraint of the project, the program does not incorporate feedback which ensures that the block is within the claw during the pickup phase of the program. This feedback would have ensured 100% efficiency of the program picking up the block.
 The resulting structure yields the letters UHI as required. The horizontal blocks placed closed to one another and are nearly perfectly straight. However, the middle blocks of the letters have gaps between them. This was to prevent pushing the previously placed blocks for the letter. The resulting structure 85% of what is considered ideal, as seen in Figure 4. 
