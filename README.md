# Traffic Light Detection, Depth Estimation, and Decision Making Algorithm

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
<!--[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]-->



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="gif/efficientDet.gif" alt="Logo" width="600" height="600">
  </a>

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#setup-the-environment">Setup the Environment</a></li>
        <li><a href="#scripts"> Execute Scripts</a></li>
      </ul>
    </li>
    <li><a href="#license">License</a></li>

  </ol>
</details>



<!-- ABOUT THE PROJECT -->
# About The Project

Traffic lights are control flow devices that guarantee security for drivers. Due to the lack of concentration and slow reaction from 
the drivers, this might lead into car accidents. Therefore, this project aims to contribute to with the implementation of a ***Emergency
 Brake Driver Assitant System from Traffic Lights using Deep Learning and a Fuzzy Logic Decision-Making Algorithm*** which will be computed 
 using a [Nvidia Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2-developer-kit) and a [ZED Stereo Camera](https://www.stereolabs.com/zed/). 

This project is the second part of my master thesis from Tecnologico de Monterrey, the name of the thesis is ***Emergency Brake Driver Assitant 
System from Traffic Lights using Deep Learning and a Fuzzy Logic Decision-Making Algorithm***. This second part consist of using the trained model 
from the first part and perform inference using the Jetson TX2, estimate the depth of the traffic light using the ZED stereo camera, and perform a 
fuzzy logic decision-making algorithm using the distance and the traffic light state, the output of the fuzzy system is a value from 0 to 1 that is 
considered to be the ***brake signal***. 

# 1. Built with
## 1.1. Environment
The project was built using:
* Tensorflow 2.3.1
* Bazel 3.1.0
* Tensorflow Addons 0.13.0
* Jetpack 4.5.1 (L4T.R32.5.1)
* CUDA 10.2
* cuDNN 8.0
* Python 3.6.9
* Ubuntu 18.04 LTS
* ROS2 Eloquent
* ZED SDK for Jetpack 4.5 (version 3.5)

# 2. Getting Started
## 2.1. Setup the Environment
In this section we provide a full guide for the installation of the dependecies needed to execute the 
***Emergency Brake Driver Assitant System from Traffic Lights using Deep Learning and a Fuzzy Logic 
Decision-Making Algorithm*** on the Jetson TX2. Please follow the order of the installation guide:
1. Jetpack 4.5.1 (L4T.R32.5.1) Installation.
1. ZED SDK Installation.
2. ROS2 Installation.
3. ROS2 Wrapper Installation.
4. Tensorflow, Bazel, Tensorflow Addons, Tensorflow Object Detecion Api Installtions.

### 2.1.1. Jetpack Installation
For this project, I used the [Jetpack 4.5.1 (L4T.R32.5.1)](https://developer.nvidia.com/jetpack-sdk-45-archive). Please follow the instructions provided by [Jethacks](https://www.youtube.com/watch?v=s1QDsa6SzuQ&t=441s).

### 2.1.2. ZED SDK Installation
Download the installer for the [zed sdk stereo camera for jetson](https://download.stereolabs.com/zedsdk/3.5/jp45/jetsons). Then, make it executable and run the installer (yes to everything)

To execute the sample related to the zed python library, execute them as follows:

```OPENBLAS_CORETYPE=ARMV8 python3 ```  According to [Errors](https://forums.developer.nvidia.com/t/illegal-instruction-core-dumped-xavier/166278/3)

### 2.1.3. ROS2 Installation 
To install ROS2 Eloquent, follow the instructions provided in the following [post](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html) 

### 2.1.4. ROS2 Wrapper Installation
First [git clone the repo](https://github.com/stereolabs/zed-ros2-wrapper/tree/master) and checkout to the eloquent branch. Once there colcon build (as specified in the master brach "only the colcon build").

### 2.1.5. Tensorflow, Bazel, Tensorflow Addons, Tensorflow Object Detecion Api Installation
There are two ways to install the Tensorflow, Bazel, Tensorflow Addons, Tensorflow Object Detecion Api. Simply execute the ```setup.py``` which contains all the dependencies of the project or by manually installing the dependecies provided in the ```Manual Installation.txt```.

```python setup.py```

## 2.2. Verify the Installation 
Once the Installation is completed check each component:

* Execute the ```./ZED_Explorer``` executable from the ZED SDK.
* Execute ```ros2 launch zed_wrapper zed.launch.py``` and ```rviz2``` and check the topics sent from the zed in rviz.
* Execute ```python3 -c 'import tensorflow as tf; print(tf.__version__)'``` it should display ```2.3.1```
* Execute ```bazel --version``` it should display ```3.1.0```
* Execute ```python3 -c 'import tensorflow_addons as tfa; print(tfa.__version__)'``` it should display ```0.13.0```
* Execute ```python3 object_detection/builders/model_builder_tf2_test.py``` in the tensorflow object detection api and no errors should be displayed.

# 3. Workspace Structure
The folder structure of the project can be seen as follows:

```
├── fuzzy_logic_design              # Test different Fuzzy Logic algorithms candidates
│   ├── TL_Decision_Making.ipynb    
├── dev_ws                          # ROS2 Workspace for the ADAS 
│   ├── src                         # Contains all the ROS2 Packages
│       ├── zed-ros2-wrapper        # ROS2 Package that retrieves the information from the ZED camera and publishes on topics
│           ├── ...
│       ├── tl_perception           # ROS2 Package that performs the model inference, disntance estimation, and decision making
│           ├── label_maps          # Class label for the detection of traffic lights
│           ├── models              # Trained models for inference
│           ├── tl_perception       # Contains all the nodes of this ROS2 package
│               ├── model_inference.py     # ROS2 node that performs the model inference and publish the classes and bounding boxes
│               ├── depth_estimation.py    # ROS2 node that estimates the distance from the detected traffic light and publist the distance and class
│               ├── tl_decision_making.py  # ROS2 node that performs fuzzy logic decision making and outputs the brake signal
│               ├── ...
│           ├── ...
│       ├── tl_interfaces           # ROS2 Package that stores all the custom interfaces (messages) of the system
│           ├── msg                 
│               ├── TLPredictions.msg      # Custom Interface for model_inference.py (publisher)
│               ├── ClassDistanceTL.msg    # Custom Interface for depth_estimation.py (publisher)
│           ├── ... 
│   ├── ...
├── Manual_Installation.txt         # Manual Installation of the dependencies to setup the environment
├── setup.py                        # Automatic Installation of the dependencies to setup the environment
├── README.md                       # Contains all the information of the project
└── ...
```

***Important:*** The ***Emergency Brake Driver Assitant System from Traffic Lights using Deep Learning and a Fuzzy Logic Decision-Making Algorithm***
only involves the files inside the ```/dev_ws/src```.

# 4. ROS2 Workspace
The project 3 ROS2 packages (One package is for the custom messages). The ROS2 packages interact as follows:


Each of the packages are described in more detail below:

## 4.1. zed-ros2-wrapper package
This package is responsible to publish all the information of the ZED Stereo camera such as: RGB images from the left and right lens, depth map, and point
cloud information.

## 4.2. tl_perception package
This package loads the trained model and performs inference, estimates the distance of the traffic light, and takes decision bases on the previous information. 
the ```model_inference.py``` takes as input the ```zed/left/rgb``` and outputs ```estimation/classes_distances```, which is  the information from the bounding boxes 
and the classes detected using the ZED Stereo Camera ```TLPredictions.msg```.  Then the ```depth_estimation.py``` estimates the distance by subscribing to the 
```/zed/zed_node/depth/depth_registered``` and to the ```estimation/classes_distances```, with this information it estimates the distance using the bounding box cordinates
and the depth map generated by the ZED Stereo Camera. Finally, the ```tl_decision_making.py``` node subscribes to the ```estimation/classes_distances``` and evaluates the distance
and tl states in the fuzzy logic system and provides the ***brake_signal*** in the ```fl_dm/output``` topic.


## 4.3. tl_interfaces package
This package contains all the custom messages required for the ADAS. These are used in the ```TL_Perception``` package.

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt

