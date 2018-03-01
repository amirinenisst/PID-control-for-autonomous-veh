# PID-control-for-autonomous-veh
Considered a kinematic model which performs well and ignores elements like gravity, tire forces etc.
State of the vehicle is given by [x,y,ψ,v] where x and y gives the position, ψ gives the orientation and v is the velocity as the vehicle is in motion.

δ is the steering angle, which is one of the actuated inputs given to the vehicle. Throttle and break are considered together as another actuated input ranging from -1 to 1 where -1 is complete break and +1 is complete acceleration.
 
Using pure pursuit algorithm, we can control the steering. For this we need to determine the current position of the vehicle. Then we need to determine the path point closest to the vehicle. We should convert goal point to vehicle coordinates. Calculation of the curvature is required and the vehicle steering is set to that curvature. Updating the position each time and repeating the steps.
PID control is used for controlling the speed. (This is the function which is written inside the algorithm class)
