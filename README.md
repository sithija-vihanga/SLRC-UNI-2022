# Multifunctional Robot with Arduino Mega, IR Sensors, and Color Detection
![Robot 2](https://github.com/sithija-vihanga/SLRC-UNI-2022/assets/106132194/268a2065-6e54-4442-b7e9-5ce4e8a4b1ea) ![Robot 1](https://github.com/sithija-vihanga/SLRC-UNI-2022/assets/106132194/78efc6fd-752b-4dd8-827c-03334113d0c2)



## Overview
Welcome to the repository of our multifunctional robot, a versatile project developed using Arduino Mega board and an array of sensors. This robot is designed to excel in various tasks, including solving the classic Tower of Hanoi problem, expertly following arrows, skillfully navigating along walls, precisely tracking lines, and detecting colors of different boxes. The robot's capabilities showcase the power of Arduino Mega, IR sensors, magnetometer, and a color sensor, enabling it to tackle a wide range of challenges.

## Hardware Components
The robot's hardware setup comprises the reliable Arduino Mega board, chosen for its computational power and versatility. To excel in its diverse tasks, the robot is equipped with 12 IR sensors, enabling accurate line detection and 2 color sensors are placed for box  detection and 7 ultrasonic sensors are used for obstacle avoidance, and wall following. Additionally, a magnetometer is utilized to continuously update the robot's orientation and direction during navigation.

The incorporation of a color sensor is instrumental in achieving precise color box detection, crucial for tasks involving distinguishing different boxes based on their colors. The combination of these hardware components empowers the robot to accomplish its multifunctional objectives with finesse.

## Tower of Hanoi Algorithm
Our robot exhibits remarkable problem-solving abilities, skillfully tackling the classic Tower of Hanoi puzzle. The objective of this task is to move three boxes of different sizes between three white boxes, ultimately building a tower in a designated white box and then transferring it to another specified white box during the start of the competition.

### Color Scanning and Box Detection
The robot starts the Tower of Hanoi task by conducting a comprehensive color scanning cycle. During this phase, it carefully scans all the boxes and gathers essential information about their positions and colors. To improve color accuracy, the robot intelligently lowers its gripper during scanning, allowing the color sensor to obtain precise readings from the white boxes and the differently colored boxes.

### Initial Tower Construction
Once the scanning is complete, the robot begins constructing the tower in the designated white box. Based on user input from a dip switch, the robot identifies the largest box and moves it to the first white box. The advanced IR sensors play a crucial role in this process, ensuring precise box grabbing and placement.

### Solving the Tower of Hanoi
Adhering to the precise rules of the Tower of Hanoi puzzle, the robot proceeds with the movement of the remaining boxes. It ensures that a larger box is never placed on top of a smaller one, guaranteeing a valid solution. The robot's navigation capabilities, powered by IR sensors and the magnetometer, enable it to move efficiently and achieve the optimal solution to the Tower of Hanoi problem.

### Final Tower Placement
After successfully constructing the tower in the designated white box, the robot proceeds to transfer the tower to the specified destination white box, completing the competition task. The precise movements, combined with the color sensor's accuracy in detecting box colors, ensure a smooth and error-free transfer of the tower.

The Tower of Hanoi task demonstrates the robot's prowess in problem-solving, navigation, and color detection. Its ability to tackle this classic puzzle showcases the integration of advanced hardware components and a well-crafted algorithm. The multifunctional robot proves its versatility and intelligence, making it a fascinating project for robotics enthusiasts and problem-solving enthusiasts alike.

## Maze Solving



We employed a straight-line following algorithm to navigate through the maze. The journey began at the white square, and we followed a curved path until we reached the green box, which served as the starting point for the maze-solving process. Utilizing the IR sensor panel, we detected the path's lines, and a color sensor at the bottom helped us identify the colors of the squares.

To successfully solve the maze, we implemented the "Always Left" method. When the robot entered the red square, it signaled the completion of the maze-solving process, and the robot exited the line maze.

The straight-line following algorithm, along with our precise sensor setup, allowed us to effectively and efficiently navigate the maze and reach the desired endpoint.

![1](https://github.com/sithija-vihanga/SLRC-UNI-2022/assets/105491340/4b4b8492-ca1d-4b90-9d61-6eac154a4935)


## Arrow Following Task
In the arrow-following task, our robot entered an elevated black arena with white arrows on the floor. The arena was designed with several arrow paths, except for the required arrow path, to challenge the robot's navigation skills. The objective was to follow the white arrows and reach the finishing line. To accomplish this, we equipped the robot with 12 IR sensors to identify the arrows accurately.

![Arrow Following Arena](https://github.com/sithija-vihanga/SLRC-UNI-2022/assets/106132194/b0486652-4ed3-4f3a-8cdb-bf8a5d5204b2)

For successful navigation, we implemented a modified PID control algorithm, allowing the robot to make precise movements along the designated arrow paths. Additionally, we utilized a combination of reverse and forward motions to adjust the robot's orientation and alignment with the arrows. The arena's entrance and exit were designed as ramps with line-following segments, ensuring a smooth transition for the robot into and out of the challenge area.

## Wall Following and Line Following
In the wall-following task, the robot demonstrates its ability to autonomously follow the walls of a defined path. The 12 IR sensors are instrumental in detecting the walls and precisely navigating along them. Utilizing this skill, the robot can confidently explore its environment while maintaining a constant distance from the walls.

Additionally, the robot is equipped with specialized line-following capabilities. Through IR sensors, the robot accurately tracks lines on the floor, adhering to their paths with precision. This skill proves valuable in various applications, from industrial automation to tracking lines on complex surfaces.

## Color Box Detection
With the help of the integrated color sensor, the robot showcases its ability to detect and differentiate between boxes based on their colors. During the scanning cycle, the robot skillfully lowers its gripper for improved color readings from the boxes and stores this information for further processing. Also robot consist of another color sensor at the bottom of the chasis to detect any colored boxes drawn on the arena. This capability is highly beneficial in scenarios where box color plays a critical role in decision-making or sorting tasks.

## Code Naming Convention and Task Calibration

In the code, we adhere to a consistent and systematic naming convention to distinguish variables and functions associated with each specific task. Each task is uniquely identified by a corresponding prefix, enabling clear differentiation within the codebase. For instance, "TH" denotes variables and functions related to the Tower of Hanoi task, "Mz" for maze solving, "AF" for arrow following, and so on. This intuitive naming approach ensures clarity and ease of identification, facilitating efficient code maintenance and development.

Additionally, the variable "helpingStage" plays a pivotal role in calibrating the robot's behavior and task execution. By manipulating this variable, users can dynamically enable or disable specific tasks, thereby tailoring the robot's behavior to focus solely on the mentioned task. This dynamic control empowers experimentation and fine-tuning of the robot's performance for each task independently.

The thoughtful implementation of this naming convention and the use of "helpingStage" add a layer of flexibility and efficiency to the robot's codebase. It facilitates seamless task isolation, allowing developers to observe the robot's prowess in Tower of Hanoi, maze solving, arrow following, or any other specific task with ease. This adaptable approach ensures a versatile and rewarding robotics experience, encouraging exploration and experimentation with the robot's multifunctional capabilities.

## Usage Instructions
To explore the multifunctional capabilities of our robot, follow the comprehensive setup instructions provided in this repository's documentation. Assemble the hardware components, upload the meticulously crafted Arduino code to the board, and connect the sensors appropriately. Once the robot is ready, it can autonomously perform tasks such as Tower of Hanoi, arrow following, wall following, line following, and color box detection, depending on the task selected.

## Contribution and Improvements
As an open-source project, we invite contributions from the community. If you have ideas to enhance the robot's navigation, optimize algorithms, or improve the color detection mechanism, feel free to submit pull requests. Additionally, bug reports and feedback are valuable for further refining the robot's performance.

Join us in exploring the possibilities of robotics and problem-solving! Let's collaborate to build innovative, multifunctional robots. Happy exploring and robot-building! 🤖💡


