# Multifunctional Robot with Arduino Mega, IR Sensors, and Color Detection

![Robot Demo](images/robot_demo.gif)

## Overview
Welcome to the repository of our multifunctional robot, a versatile project developed using Arduino Mega board and an array of sensors. This robot is designed to excel in various tasks, including solving the classic Tower of Hanoi problem, expertly following arrows, skillfully navigating along walls, precisely tracking lines, and detecting colors of different boxes. The robot's capabilities showcase the power of Arduino Mega, IR sensors, magnetometer, and a color sensor, enabling it to tackle a wide range of challenges.

## Hardware Components
The robot's hardware setup comprises the reliable Arduino Mega board, chosen for its computational power and versatility. To excel in its diverse tasks, the robot is equipped with 12 IR sensors, enabling accurate line detection and 2 color sensors are placed for box  detection and 7 ultrasonic sensors are used for obstacle avoidance, and wall following. Additionally, a magnetometer is utilized to continuously update the robot's orientation and direction during navigation.

The incorporation of a color sensor is instrumental in achieving precise color box detection, crucial for tasks involving distinguishing different boxes based on their colors. The combination of these hardware components empowers the robot to accomplish its multifunctional objectives with finesse.

## Tower of Hanoi Algorithm
Our robot exhibits remarkable problem-solving abilities, skillfully tackling the classic Tower of Hanoi puzzle. The algorithm begins with a comprehensive color scanning cycle, where the robot gathers essential information about box positions and colors. Once the scanning is complete, the robot strategically moves the largest box to the first white box, based on user input from a dip switch.

Adhering to the precise rules of the Tower of Hanoi puzzle, the robot orchestrates the movement of the remaining boxes with utmost precision, never placing a larger box on top of a smaller one. The advanced IR sensors and magnetometer enable the robot to navigate efficiently, ensuring an optimal solution to the Tower of Hanoi problem.

## Arrow Following Task
The robot's navigation capabilities are put to the test in the arrow-following task. Placed in an elevated black arena adorned with white arrows, the robot must successfully follow the arrows and reach the finishing line. To achieve this, the robot utilizes 11 IR sensors positioned strategically to identify and interpret the arrow directions.

A modified PID control algorithm facilitates precise movements along the designated arrow paths. The robot expertly employs a combination of forward and reverse motions to align its orientation with the arrows, skillfully navigating through the challenging arena. Smooth transitions into and out of the challenge area are ensured by incorporating ramps and line-following segments at the arena's entrance and exit.

## Wall Following and Line Following
In the wall-following task, the robot demonstrates its ability to autonomously follow the walls of a defined path. The 12 IR sensors are instrumental in detecting the walls and precisely navigating along them. Utilizing this skill, the robot can confidently explore its environment while maintaining a constant distance from the walls.

Additionally, the robot is equipped with specialized line-following capabilities. Through IR sensors, the robot accurately tracks lines on the floor, adhering to their paths with precision. This skill proves valuable in various applications, from industrial automation to tracking lines on complex surfaces.

## Color Box Detection
With the help of the integrated color sensor, the robot showcases its ability to detect and differentiate between boxes based on their colors. During the scanning cycle, the robot skillfully lowers its gripper for improved color readings from the boxes and stores this information for further processing. Also robot consist of another color sensor at the bottom of the chasis to detect any colored boxes drawn on the arena. This capability is highly beneficial in scenarios where box color plays a critical role in decision-making or sorting tasks.

## Usage Instructions
To explore the multifunctional capabilities of our robot, follow the comprehensive setup instructions provided in this repository's documentation. Assemble the hardware components, upload the meticulously crafted Arduino code to the board, and connect the sensors appropriately. Once the robot is ready, it can autonomously perform tasks such as Tower of Hanoi, arrow following, wall following, line following, and color box detection, depending on the task selected.

## Contribution and Improvements
As an open-source project, we invite contributions from the community. If you have ideas to enhance the robot's navigation, optimize algorithms, or improve the color detection mechanism, feel free to submit pull requests. Additionally, bug reports and feedback are valuable for further refining the robot's performance.

Join us in exploring the possibilities of robotics and problem-solving! Let's collaborate to build innovative, multifunctional robots. Happy exploring and robot-building! 🤖💡

(Note: The above description highlights the multifunctional aspects of the robot, covering Tower of Hanoi, arrow following, wall following, line following, and color box detection. You can further refine and expand each task's description as needed in separate sections of the README.)
