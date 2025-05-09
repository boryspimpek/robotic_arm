# robot_arm/config.py

# Arm segment lengths (in mm)
L1 = 120  
L2 = 120
L3 = 70  

# Servo IDs for different parts of the robotic arm
base = 1        
shoulder = 2   
elbow = 3       
wrist = 4       
gripper = 5     

# Communication settings
port = "/dev/ttyACM0"  
baudrate = 1000000     

# Motion settings
st_speed = 500         
sc_speed = 500         
st_acc = 25            
sc_acc = 25            
angle_limits = (0, 180)  

# Servo trims (adjustments for alignment)
trims = {
    base: 0,        
    shoulder: 0,   
    elbow: 0,       
    wrist: 5        
}
