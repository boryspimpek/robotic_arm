import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import time
import math
import matplotlib
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
import serial

my_chain = ikpy.chain.Chain.from_urdf_file("robo.urdf",active_links_mask=[False, True, True, True, True])


target_position = [ 0., 0.05, 0.125]

target_orientation = [0, 0, 0]

ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
# ik = my_chain.inverse_kinematics(target_position)

angles_deg = list(map(lambda r: math.degrees(r), ik.tolist()))
formatted_angles = [f"{angle:.2f}" for angle in angles_deg]
print("The angles of each joint (in degrees) are:", formatted_angles)

matplotlib.use('tkagg')
fig, ax = plot_utils.init_3d_figure()
fig.set_figheight(9)  
fig.set_figwidth(13)  

my_chain.plot(ik, ax, target=target_position)

plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
ax.set_zlim(0, 0.6)

ax.view_init(elev=40, azim=60)  # Opcjonalnie ustaw widok kamery

plt.show()
