## Arrow Following Task
In the arrow-following task, our robot entered an elevated black arena with white arrows on the floor. The arena was designed with several arrow paths, except for the required arrow path, to challenge the robot's navigation skills. The objective was to follow the white arrows and reach the finishing line. To accomplish this, we equipped the robot with 12 IR sensors to identify the arrows accurately.

<figure style="float: left; width: 200px;">
  <img src="https://github.com/sithija-vihanga/SLRC-UNI-2022/assets/106132194/49de2ae8-6a63-41a3-a495-515373c086aa" alt="Arrow Following Arena">
  <figcaption style="color: blue; font-size: 12px; text-align: center;">Arrow Following Arena</figcaption>
</figure>



For successful navigation, we implemented a modified PID control algorithm, allowing the robot to make precise movements along the designated arrow paths. Additionally, we utilized a combination of reverse and forward motions to adjust the robot's orientation and alignment with the arrows. The arena's entrance and exit were designed as ramps with line-following segments, ensuring a smooth transition for the robot into and out of the challenge area.

