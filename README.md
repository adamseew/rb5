
![RB5 logo](./rb5_logo.svg)

# RB5 wheeled experimental robotic platform ROS2 and ROS packages

In this git repo, you can find the source code of the ROS2 and ROS packages for the RB5 experimental robotic platform. 

It constitutes the supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/Vflmh6LTo6A" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Installation

Install all the ROS2 and ROS packages in your local workspace.

In this repository, you will find the **ground robot** ROS2 package, which contains the code that runs on the companion computer onboard RB5, i.e., an <a rel="jetson" href="https://www.seeedstudio.com/reComputer-J2021-p-5438.html?">NVIDIA (C) Jetson (R) NX</a>.

## Usage

Clone the repository in your local workspace and compile the workspace.

To run for instance the remote control via LoRa long-range low-power communication technology from the internet-of-things domain, you will need the `chw_bs_comm` node (communication with a remotely located base station) and the `chw_ctl_comm` node (communication with the microcontroller).

## Hardware

Follows a list of the necessary hardware for the approach. Motors, motor drives, and the battery are not listed.

| HW | Scope | Amount |
|-|-|:-:|
| <a rel="jetson" href="https://www.pjrc.com/store/teensy40.html">Teensy (R) 4.0 Development Board</a> | Low level processing, microcontroller | 1x |
| <a rel="jetson" href="https://www.seeedstudio.com/reComputer-J2021-p-5438.html?">NVIDIA (C) Jetson (R) NX</a> |  High level processing, companion computer onboard | 1x |
| <a rel="lora" href="https://ronoth.com/products/lostik">LoRa bundle with the RN2903 module</a> | Communication on long distances | 2x |
| <a rel="RGB-D" href="https://www.intel.com/content/www/us/en/products/sku/189347"> Intel (R) AX200 network card</a> | Communication on short distances | 1x |
| <a rel="RGB-D" href="https://www.intelrealsense.com/depth-camera-d435/">Intel (R) RealSense (TM) D435 RGB-D camera</a> | Visual sensing, depth perception, SLAM | 1x |


## License
<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.