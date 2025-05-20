# robot_arm/config.py

# Arm segment lengths (in mm)
L1 = 120  
L2 = 120
L3 = 110  

# Servo IDs for different parts of the robotic arm
base = 1        
shoulder = 2   
elbow = 3       
wrist = 4       
gripper = 5     

# Communication settings
port_bus = "/dev/ttyACM0" 
port_ps4 = "/dev/input/js0" 
baudrate = 1000000     

# Motion settings
st_speed = 500         
sc_speed = 500         
st_acc = 80            
sc_acc = 25      

# angle_limits = (0, 180)      
base_angle_limits = (0, 180)
shoulder_angle_limits = (0, 180)
elbow_angle_limits = (0, 290)
wrist_angle_limits = (0, 260)  

# Servo trims (adjustments for alignment)
servo_trims = {
    base: 2.5,
    shoulder: 0,
    elbow: 0,
    wrist: 4
}

