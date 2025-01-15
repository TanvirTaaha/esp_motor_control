# Differntial motor controller for esp32 using serial communication
- This is made to read the encoder values from two motors and send over serial while receiving motor target commands from serial.
- Uses esp32 pulse counter for counting encoder pulses. Right now doesn't support magnetic angle sensors.
- Implements PID for feedback control.
The PID part is taken from [Josh Newans](https://github.com/joshnewans/ros_arduino_bridge.git) from [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) YT channel.
