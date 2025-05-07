from ikpy.chain import Chain
import numpy as np

# Wczytanie modelu z URDF
robot_chain = Chain.from_urdf_file("robo.urdf", base_elements=["base_link"], last_link_vector=[0, 0, 0.06])

# Cel w przestrzeni 3D
target_position = [0.1, 0.1, 0.3]

# Oblicz kąt dla każdego stawu
angles = robot_chain.inverse_kinematics(target_position)

print("Joint Angles:", angles)
