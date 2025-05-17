# robot_arm/config.py

# Arm segment lengths (in mm)
L1 = 120  
L2 = 120
L3 = 100  

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
base_angle_limits = (0, 180)
shoulder_angle_limits = (0, 180)
elbow_angle_limits = (0, 240)
wrist_angle_limits = (0, 220)  

# Servo trims (adjustments for alignment)
trims = {
    base: 0,        
    shoulder: 0,   
    elbow: 0,       
    wrist: 5        
}
