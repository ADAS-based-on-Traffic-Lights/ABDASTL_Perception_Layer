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

<!--[![Product Name Screen Shot][product-screenshot]](https://example.com -->

Traffic lights are control flow devices that guarantee security for drivers. Due to the lack of concentration from the drivers, which might cause car accidents. Therefore, this project aims to contribute to with the detection an classificaction of traffic lights using state-of-the-art Convolutional Neural Network. 

This project is the second part of my master thesis from Tecnologico de Monterrey, the name of the thesis is ***Emergency Brake Driver Assistant System from Traffic Lights using Deep Learning and Fuzzy Logic.***. This second part consist of using the trained model from the first part and perform inference using the Jetson TX2, estimate the depth of the traffic light using the ZED stereo camera, and perform a decision-making algorithm using fuzzy logic. 

# Built with
## Environment
The project was built using:
* Tensorflow 2.3.1
* Bazel 3.1.0
* Tensorflow Addons 0.13.0
* Jetpack 4.5 (L4T.R32.5.0)
* CUDA 10.2
* cuDNN 8.0
* Python 3.6.9
* Ubuntu 18.04 LTS
* ROS2 Eloquent
* ZED SDK for Jetpack 4.5 (version 3.5)

# Getting Started
## Setup the Environment
In this section we provide a full guide for the installation of the dependecies needed to execute the 
Emergency Brake Driver Assistant System from Traffic Lights on the Jetson TX2. Please follow the order of 
the installation guide:
1. Jetpack 4.5 (L4T.R32.5.0) Installation.
1. ZED SDK Installation.
2. ROS2 Installation.
3. ROS2 Wrapper Installation.
4. Tensorflow, Bazel, Tensorflow Addons, Tensorflow Object Detecion Api Installtions.

### Jetpack Installation
For this project, I used the [Jetpack 4.5 (L4T.R32.5.0)](https://developer.nvidia.com/jetpack-sdk-45-archive). Please follow the instructions provided by [Jethacks](https://www.youtube.com/watch?v=s1QDsa6SzuQ&t=441s).

### ZED SDK Installation
Download the installer for the [zed sdk stereo camera for jetson](https://download.stereolabs.com/zedsdk/3.5/jp45/jetsons). Then, make it executable and run the installer (yes to everything)

To execute the sample related to the zed python library, execute them as follows:

```OPENBLAS_CORETYPE=ARMV8 python3 ```  According to [Errors](https://forums.developer.nvidia.com/t/illegal-instruction-core-dumped-xavier/166278/3)

### ROS2 Installation 
To install ROS2 Eloquent, follow the instructions provided in the following [post](https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html) 

### ROS2 Wrapper Installation
First [git clone the repo](https://github.com/stereolabs/zed-ros2-wrapper/tree/master) and checkout to the eloquent branch. Once there colcon build (as specified in the master brach "only the colcon build").

### Tensorflow, Bazel, Tensorflow Addons, Tensorflow Object Detecion Api Installation
There are two ways to install the Tensorflow, Bazel, Tensorflow Addons, Tensorflow Object Detecion Api. Simply execute the ```setup.py``` which contains all the dependencies of the project or by manually installing the dependecies provided in the ```Manual Installation.txt```.

```python setup.py```

## Verify the Installation 
Once the Installation is completed check each component:

* Execute the ```./ZED_Explorer``` executable from the ZED SDK.
* Execute ```ros2 launch zed_wrapper zed.launch.py``` and ```rviz2``` and check the topics sent from the zed in rviz.
* Execute ```python3 -c 'import tensorflow as tf; print(tf.__version__)'``` it should display ```2.3.1```
* Execute ```bazel --version``` it should display ```3.1.0```
* Execute ```python3 -c 'import tensorflow_addons as tfa; print(tfa.__version__)'``` it should display ```0.13.0```
* Execute ```python3 object_detection/builders/model_builder_tf2_test.py``` in the tensorflow object detection api and no errors should be displayed.

# Explanation of the ROS2 Packages
The project Consists of 5 ROS packages (One package is for the custom messages). The ROS2 packages interact as follows:

Each packages is described in detail bellow:

## zed-ros2-wrapper package
This package contains all the information from the ZED Stereo camera. It publishes raw, rgb, depth map, point cloud information from the camera.

## tl_perception package
This package loads the trained model and performs inference. It takes as inputs the ```zed/left/rgb``` and outputs (publishes) the information from the bounding boxes and the classes detected
using the ZED Stereo Camera (TLPredictions.msg).

## tl_interfaces package
This package contains the custom messages from the project. 

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


