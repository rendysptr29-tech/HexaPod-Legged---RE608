import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Panjang kaki
L1 = 4
L2 = 8
L3 = 10

fig = plt.figure(figsize=(10,6))
ax = fig.add_subplot(111, projection='3d')

trail_x, trail_y, trail_z = [], [], []

# IK (Geometri)

def inverse_kinematics(x, y, z):

  
    theta1 = np.arctan2(y, x)

    r = np.sqrt(x**2 + y**2) - L1
    D = (r**2 + z**2 - L2**2 - L3**2)/(2*L2*L3)

    D = np.clip(D, -1.0, 1.0)
    theta3 = np.arctan2(-np.sqrt(1-D**2), D)

    theta2 = np.arctan2(z, r) - np.arctan2(L3*np.sin(theta3), L2 + L3*np.cos(theta3))

    return theta1, theta2, theta3

# FK

def forward_kinematics(theta1, theta2, theta3):

    x1 = L1*np.cos(theta1)
    y1 = L1*np.sin(theta1)
    z1 = 0

    x2 = x1 + L2*np.cos(theta1)*np.cos(theta2)
    y2 = y1 + L2*np.sin(theta1)*np.cos(theta2)
    z2 = L2*np.sin(theta2)

    x3 = x2 + L3*np.cos(theta1)*np.cos(theta2+theta3)
    y3 = y2 + L3*np.sin(theta1)*np.cos(theta2+theta3)
    z3 = z2 + L3*np.sin(theta2+theta3)

    return (x1,y1,z1),(x2,y2,z2),(x3,y3,z3)

def update(frame):

    ax.clear()

    t = frame * 0.05

    # TARGET (SPATIAL TRAJECTORY)
    target_x = 8 + 3*np.cos(t)
    target_y = 3*np.sin(t)
    target_z = -10 + 2*np.sin(t)

    # IK - sudut joint
    theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, target_z)

    # FK - posisi joint
    (x1,y1,z1),(x2,y2,z2),(x3,y3,z3) = forward_kinematics(theta1, theta2, theta3)

    trail_x.append(x3)
    trail_y.append(y3)
    trail_z.append(z3)

    if len(trail_x)>50:
        trail_x.pop(0)
        trail_y.pop(0)
        trail_z.pop(0)

    ax.plot([0,x1],[0,y1],[0,z1],linewidth=5,color='blue')
    ax.plot([x1,x2],[y1,y2],[z1,z2],linewidth=5,color='orange')
    ax.plot([x2,x3],[y2,y3],[z2,z3],linewidth=5,color='purple')
    ax.scatter(x3,y3,z3,color='red',s=80)
    ax.scatter(target_x,target_y,target_z,color='green',marker='x',s=80)
    ax.plot(trail_x,trail_y,trail_z,'r--')

    table_data = [
        ["θ1 (Coxa)",f"{np.degrees(theta1):.1f}°"],
        ["θ2 (Femur)",f"{np.degrees(theta2):.1f}°"],
        ["θ3 (Tibia)",f"{np.degrees(theta3):.1f}°"],
        ["Target X",f"{target_x:.2f}"],
        ["Target Y",f"{target_y:.2f}"],
        ["Target Z",f"{target_z:.2f}"],
        ["End X",f"{x3:.2f}"],
        ["End Y",f"{y3:.2f}"],
        ["End Z",f"{z3:.2f}"]
    ]

    ax.table(
        cellText=table_data,
        colLabels=["Parameter","Value"],
        bbox=[-0.5,0.5,0.4,0.45]
    )

    # axis
    ax.set_xlim(-15,15)
    ax.set_ylim(-15,15)
    ax.set_zlim(-20,5)

    ax.set_box_aspect([1,1,1])
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_title("Forward + Inverse Kinematics (Spatial & Geometric)")

ani = FuncAnimation(fig, update, frames=500, interval=30)
plt.show()