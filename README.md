# ArUCo_Mobile_Robot

## Introduction
This project aims to design and implement a wheeled mobile robot, to address the challenges of missed parcel deliveries and theft in outdated delivery systems and infrastructure. Using computer vision techniques and OpenCV, the robot autonomously detects parcels, navigates to delivery points, and avoids obstacles. The project involves the implementation of computer vision algorithms like ArUco marker detection and obstacle avoidance using Canny edge detection. Results demonstrate successful localization, obstacle avoidance, and control of robot movement. The significance lies in providing a practical solution to the persistent problem of missed deliveries, improving efficiency, and enhancing the security of parcel delivery systems. This project's capabilities showcase the potential of robotics and computer vision in modernizing delivery infrastructure and enhancing user experience.

## Methodology

### Initial Design Phase

The project began with the identification of the primary objective: utilizing Jetson Nano and a camera to enhance university life, leading to the concept of a mobile robot for parcel retrieval. The initial design outlined essential functionalities, including movement, parcel detection using infrared sensors, and parcel delivery via a piston mechanism.

### Iterative Design Refinement

Following consultation with the professor, the design underwent iterative refinement. A rectangular body with wheels, a push piston, and Jetson Nano with cameras was conceptualized. Subsequently, adjustments were made to accommodate internal components, necessitating a larger body size and relocation of the cameras.

### Component Selection and Ordering

Informed by design iterations, components were selected, and orders were placed. Modifications were made during this stage due to budget constraints, including replacing pistons with a screw system and switching to servos for motor control. The motor driver was also changed to PCA9685 for servo control.

### Acrylic Sheet Design and Fabrication

The design of the robot's body involved using a 3 mm acrylic sheet with specific compartments for housing components. Each acrylic piece was meticulously designed with grooves and holes for component clearance and assembly ease. Laser cutting was employed for fabricating the acrylic sheets.

### Camera

Certain components, such as the camera holder and parts for the piston screw system, were designed and 3D printed. Multiple iterations were conducted to ensure proper fit and functionality, particularly for the camera holder. In the end we changed to a webcam because all the cameras were broken.

### Assembly Process

Prior to assembly, all parts were inspected for proper fit within the acrylic grooves. Sanding was employed to adjust acrylic pieces as necessary. Assembly commenced by gluing together acrylic parts, starting with the camera arm, followed by the sides and connecting all components. Servo motors were then attached to the body, and wheels were connected to the motors, along with the caster wheel to the middle acrylic piece.

### Implementing ArUco Marker Detection for Localization

In order to use ArUCo markers, we first need to find the calibration and distortion matrix of our camera. We can do this by using the OpenCV library to detect a chess board in numerous positions and orientations (See Fig. 1(a) and 1(b)). 

![image](https://github.com/user-attachments/assets/9ce79154-7bcd-4534-a832-430efcb69546)

Fig. 1(a). Calibration Chess Board

![image](https://github.com/user-attachments/assets/8384a748-1521-4f00-a1ff-025eaf5083cc)

Fig. 1(b). Detection of Calibration Chess Board

After generating the calibration and distortion matrix of the camera, we can now start working with the ArUCo markers. Firstly, we need to install the opencv-contrib-python, a public contribution version of OpenCV which includes the ArUCo library, onto our Jetson Nano. We then started testing with our first program. Detect.py simply captures a single frame from the camera and detects and identifies the ArUCo markers present (See Fig. 2(a)). 

![image](https://github.com/user-attachments/assets/848a4c69-d07e-4a5d-bb40-d3b58e5c85c8)

Fig. 2(a). Detection and Classification of ArUco Markers From a Still Image

We then use the calibration and distortion matrix to help us predict the locations and orientations of the ArUCo markers. In Detect2.py, we use live feed from the camera to process and detect ArUCo markers. We then display the x, y, and rotation coordinates of the camera with respect to the first ArUCo marker detected (See Fig. 2(b)).

![image](https://github.com/user-attachments/assets/56b5de40-fec8-4685-9289-53a7726e1fdc)

Fig. 2(b). Distance and Pose Estimation of ArUco Markers From a Live Video Feed

### Implementing Scanning Phase

In order for the robot to find its own position and orientation, we have decided to implement a scan phase to begin the program. We first control the robot to spin in place. While the robot is spinning, it would detect the ArUco markers which are placed in a specific way, storing the information of distance and orientation in a dictionary. The robot would keep spinning until it has detected all of the ArUco markers present. Once it has gotten all of the distance and orientation (The absolute orientation of the ArUco markers are given to the robot as an outside reference), it would then be able to calculate its current position and absolute orientation by using simple geometry. Once the robot knows its position and orientation, it would be able to calculate the angle it needs to turn in order to face the target ArUco marker and the distance from itself to the target ArUco marker.

### Implementing Obstacle Avoidance Using Canny Edge Detection

In order to determine whether there is an object on the left side, right side, or directly down the middle, we can utilize Canny edge detection to find the side which contains the most edge and the edge count is more than a certain threshold; therefore determining which side has an object. (See Fig. 3)

![image](https://github.com/user-attachments/assets/ef19a9d7-abee-4654-ae59-7371f94b8ae3)

Fig. 3. Edge Detection for Obstacle Avoidance

Implementing this concept, we first take the video feed from the camera mounted on the top of our robot and process each frame read by the program by applying the blur filter to try and remove as much noise as possible, then applying the canny filter (Both blur and Canny filter is available using OpenCV). We then split the frame into three equal sections: left, middle, and right. After that, we process the frame which has been altered by the blur and Canny filter by counting and keeping track of the amount of white pixels or edges produced by the Canny filter. Knowing the average number of white pixels in each section of the frame, we are able to set a threshold for the amount of pixels that would be classified as an object ahead. The default movement of the robot is forward; therefore, we would check the middle section for objects first. If the number of white pixels in the middle section doesn’t reach the threshold set, we would continue moving forward. On the other hand, if there is an obstacle directly ahead, the pixel count of the middle section would be over the threshold, we would then compare the pixel count of the left and right sections in order to determine the direction we should move towards. In applying this algorithm, setting the right threshold number is key, as different environments and settings could cause presents and vastly different optimal threshold numbers and obtaining a good threshold number can be done by experimenting.

An extra factor that we need to consider is the fact that the ArUco markers on the ground are also detected as edges and could possibly mess up the obstacle avoidance system. In our program, we must consider the scenario where the number of white pixels detected in the middle section of the frame goes over our threshold due to the robot detecting the ArUco marker right ahead of it. We must also adjust the threshold accordingly in order to avoid the robot misinterpreting the target ArUco marker as an obstacle.

### Controlling the Movement of the Robot

When controlling our robot to move towards the target, we need to know the speed that our robot should move at, both in the forward direction and rotationally. We can determine this from our information of the robot’s distance and angle from the target and a set travel time. Once we know the velocity of our robot, we have to translate that information into the rotational speed of our robot’s wheels. For that task, we decided to develop a simple inverse kinematics model using an analytical method. Since our robot is a differential drive wheel robot, the equations that show the relationship between the forward and rotational velocity of our robot to the rotational speed of the left and right wheel are Eq.(3) and Eq.(4) respectively.

$$\Phi_{l}' = \frac{1}{R}(V_{A} - \omega L) \longrightarrow (3)$$            
$$\Phi_{r}' = \frac{1}{R}(V_{A} + \omega L) \longrightarrow (4)$$                       

Where R is the radius of the wheel, vA is the forward velocity of the robot,  is the rotational velocity of the robot, and L is the length from the middle of the robot to the middle of the wheel.

In order to simplify the calculations and lower the amount of variations and factors that are hard to account for, we have decided to separate the movement towards the target into two phases: rotate and forward movement. This would reduce the complexity of the movement and allow for easier monitoring and debugging. 

Our robot is a wheeled mobile robot using differential drive, which means we can control the robot by setting the rotational speed of two separate wheels. We control the speed of the wheels by controlling the speed of two continuous servo motors through the PCA9685. The PCA9685 and the Adafruit_PCA9685 module allows us to control the speed and direction of the continuous servo motors using PWM signal. We must first determine the relationship between the PWM signal and the rotational speed of the motor shaft (in radians per second) in order to have accurate control of our robot. We are able to do this by experimenting with different PWM values and recording the speed of the wheel. According to documentation online, the relationship between PWM values and the rotational speed are usually linear and our experimental results do align with this finding.


## Overall Result

After assembling all of the different parts of the robot and implementing multiple computer vision algorithms and techniques by utilizing models provided by OpenCV, we are able to build a wheeled mobile robot that is able to locate and navigate to a target location while avoiding obstacles, and successfully drop of a package that is placed atop of itself at the intended location.

## Conclusion

We created this project in order to develop a proof of concept that would help solve the problem of parcel delivery. We utilize ArUco markers in combination with Canny edge detection as the main parts of our program. The ArUco markers are able to help the robot with localization, determining its own position and orientation with limited peripherals and knowledge of its surroundings. Canny edge detection is used to detect obstacles and help the robot avoid collisions. We also utilize a simple kinematics model developed using an analytical method of a differential drive mobile robot in order to precisely control the movement of our robot.

From our project, we have found that ArUco markers are great tools for localization. It is fast and easy to use, as it comes with the OpenCV contrib module and it is fully equipped with functions that greatly aid the process of finding distance and orientation. As for obstacle avoidance, using Canny edge detection is a great way to detect objects without relying on many sensors and other components. Using only the feed from the camera, the robot is able to detect objects in front of it and adjust accordingly. The Canny filter is also easy and fast to use, being implemented by the OpenCV module. However, problems can occur when implementing both ArUco markers and Canny edge detection. These two great implementations of computer visions can create problems when used together. The ArUco markers’ patterns present edges that are not from an actual obstacle that needs to be avoided. This could cause great confusion for the Canny edge detection and fixes that were implemented are situational and may not be applicable in all circumstances.

In conclusion, we were able to implement localization and obstacle avoidance in mobile robots using minimal peripherals by utilizing computer vision. The utilization of ArUco markers and Canny edge detections have great potential for further development and research. The combination of the amazing accuracy and precision that comes with ArUco markers and the ability of Canny edge detection to detect obstacles should be further explored. It is crucial to further develop better algorithms and implementations of these two concepts, maximizing its upside potential and minimizing the contradiction between the two, as these two concepts can be applicable in many fields of engineering.

