# Closed Loop Pressure Control in Fixed Volume Container with PID Controllers

This repository contains code and documentation for the project "Closed Loop Pressure Control in Fixed Volume Container with PID Controllers". 

<p float="middle">
  <img src="/doc/images/all_system.jpg" width="250" hspace="40"/>
  <img src="/doc/images/image14.jpg" width="400" "/> 
</p>


## System Description
* **Actuator:** Mini Air Compressor/Pump 
* **Plant:** Fixed Volume Container 
* **Controller:** Arduino
* **Controller Types:** On-Off Controller, PID Family
* **Sensor:** BMP180 Digital Pressure Sensor
* **Input:** Desired Pressure
* **Output:** Pressure in the Container

Block diagram of the system can be seen below:
![Block diagram of the experimental setup](/doc/images/block_diagram.png)


## System Identification
Plant was assumed to have a First Order Plus Dead Time (FOPDT) model and system identification was done using bump test (See [documentation](/doc) for more details).

## Code
Repository contains two main files one for Arduino and one for MATLAB.
* **bmp180_pid.ino:** It contains on-off controller and PID family controller implementations for Arduino. It sends data to MATLAB via serial for visualization and further processing. Output (i.e. pressure in the container) was read by BMP180 pressure sensor. 
* **pressure_control.m:** This MATLAB script gets data from Arduino via serial. It does system identification. Moreover, it calculates simulated response for the given input and visualizes simulated response as well as experimental response. 

## Results
Bump test:

![Bump test](/doc/images/bump_test.png)

On-Off Controller:

<img src="/doc/images/on_off_controller.png" width="600"/>



P Controller:

<img src="/doc/images/p_controller.png" width="600"/>



PI Controller:

![PI Controller](/doc/images/pi_controller.png)




PID Controller:

![PID Controller](/doc/images/pid_controller.png)





