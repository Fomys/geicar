#General orders
bool start false 
int8 mode 0 # 0 -> Manual ; 1 -> Autonomous ; 2 -> Steering Calibration ; 3 -> recording ; 4 -> play recording


#Manual orders
float32 throttle 0  #[0 ; 1]
float32 steer 0     #[-1 ; 1]  
bool reverse false