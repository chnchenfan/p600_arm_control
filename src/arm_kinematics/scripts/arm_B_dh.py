import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# 有4个解
def calculate_angles_all(px, py, pz, l1, l2, l3):
    # 计算 theta_1
    theta=[]#记录总共的解
    theta_1 = math.atan2(-py, -px) 
    group1=Inverse_calculation(theta_1,px, pz, l1, l2, l3)
    for i in range(len(group1)):
        theta.append(group1[i])
    theta_1 = math.atan2(py, px) 
    group1=Inverse_calculation(theta_1,px, pz, l1, l2, l3)
    for i in range(len(group1)):
        theta.append(group1[i])
    # 返回结果为列表形式
    return theta
# 函数对应theta3,theta2的两组逆解
def Inverse_calculation(theta1,px, pz, l1, l2, l3):
    theta_group=[]#记录2组解
    # 计算 A
    A = -px / math.cos(theta1)
    # 计算 B
    B = pz - l1
    # 计算 cos(theta_3)
    cos_theta_3 = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
    # 检查 cos(theta_3) 是否在 [-1, 1] 范围内
    if abs(cos_theta_3) > 1:
        print(f"Warning: cos(theta_3) = {cos_theta_3} is out of range [-1, 1].")
        cos_theta_3 = max(-1, min(1, cos_theta_3))  # 限制在 [-1, 1] 范围内
    theta_3p = math.acos(cos_theta_3)
    theta_3n = -theta_3p
    # 计算 C 和 D
    C_p = l2 + l3 * math.cos(theta_3p)
    D_p = l3 * math.sin(theta_3p)
    
    C_n = l2 + l3 * math.cos(theta_3n)
    D_n = l3 * math.sin(theta_3n)
    
    # 计算 theta_2 对应于 theta_3p 和 theta_3n
    theta_2p = math.atan2(A * C_p - B * D_p, A * D_p + B * C_p)
    theta_2n = math.atan2(A * C_n - B * D_n, A * D_n + B * C_n)
    
    # 确保所有角度在 -pi 到 pi 之间
    theta1 = restrict_angle_pi(theta1)
    theta_2p = restrict_angle_pi(theta_2p)
    theta_3p = restrict_angle_pi(theta_3p)
    theta_2n = restrict_angle_pi(theta_2n)
    theta_3n = restrict_angle_pi(theta_3n)
    # 将两组解存储到 theta_group
    theta_group.append([theta1, theta_2p, theta_3p])
    theta_group.append([theta1, theta_2n, theta_3n])
    return theta_group

def restrict_angle_pi(angle):
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle

def set_axes_equal(ax):
    # 设置坐标轴的相等比例，使得球体显示为球体，立方体显示为立方体等
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
# 标准DH表函数
def dh_matrix(alpha, a, d, theta):
    # 根据DH参数计算变换矩阵
    matrix = np.identity(4)
    matrix[0, 0] = np.cos(theta)
    matrix[0, 1] = -np.sin(theta) * np.cos(alpha)
    matrix[0, 2] = np.sin(theta) * np.sin(alpha)
    matrix[0, 3] = a * np.cos(theta)
    matrix[1, 0] = np.sin(theta)
    matrix[1, 1] = np.cos(theta) * np.cos(alpha)
    matrix[1, 2] = -np.cos(theta) * np.sin(alpha)
    matrix[1, 3] = a * np.sin(theta)
    matrix[2, 0] = 0
    matrix[2, 1] = np.sin(alpha)
    matrix[2, 2] = np.cos(alpha)
    matrix[2, 3] = d
    matrix[3, 0] = 0
    matrix[3, 1] = 0
    matrix[3, 2] = 0
    matrix[3, 3] = 1
    return matrix
#设置坐标系个数
joint_num = 3     #机械臂关节个数
CoordinatePoints_num=2+joint_num    #坐标系个数
l1 = 0.0375
l2 = 0.105
l3 = 0.18326


# 4个参数中,第一列默认为0，相当于世界坐标系的点
joints_alpha = [math.pi/2, 0, 0]
joints_a = [0, 0.105, 0.18326]
joints_d = [0.0375, 0, 0]
joints_theta = [0,math.atan(55.94/88.86),-math.atan(97.64/155.09)]

# 看DH表定义
joints_angle_init = [ 0,-math.atan(55.94/88.86)+math.pi/2, math.atan(97.64/155.09)]  #给这个时，4个坐标点在一个线上，逆解角度在这个基础上，所有角度为0；但是正解是DH表，所以需要转换下
joints_angle = [ 0,0, 0]   #转动值可修改，0默认为初始状态,以此时DH表的基础作为初始状态角度
# 机体原点
T_I = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
# 创建矩阵 T_B^0
T_B0 = np.array([
    [1, 0, 0, 0.104],
    [0, -1, 0, 0],
    [0, 0, -1, 0.10594],
    [0, 0, 0, 1]
])
T = []  # 记录所有T矩阵
T.append(T_I)
T.append(T_B0)
T_dh= []  # 记录DH表T矩阵
T_dh.append(T_I)
# 添加T矩阵
for i in range(joint_num):        
    T.append(dh_matrix(joints_alpha[i], joints_a[i], joints_d[i], joints_theta[i]+joints_angle[i]))
    T_dh.append(dh_matrix(joints_alpha[i], joints_a[i], joints_d[i], joints_theta[i]+joints_angle[i]))

# T变换矩阵相乘，计算基于机体坐标系的坐标点在世界坐标系下的坐标值
for i in range(CoordinatePoints_num-1):
    T[i+1] = np.dot(T[i], T[i+1])  
# T_dh变换矩阵相乘，计算基于DH表坐标系的坐标点，即机械臂起始坐标系
for i in range(CoordinatePoints_num-2):
    T_dh[i+1] = np.dot(T_dh[i], T_dh[i+1]) 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(CoordinatePoints_num):
    hm = T[i]
    x, y, z = hm[:3, 3]
    ax.scatter(x, y, z, c='r')

    origin = hm[:3, 3]
    x_axis = hm[:3, 0] * 0.2 + origin
    y_axis = hm[:3, 1] * 0.2 + origin
    z_axis = hm[:3, 2] * 0.2 + origin

    ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r-')
    ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g-')
    ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b-')

    if i > 0:
        prev_hm = T[i-1]
        prev_origin = prev_hm[:3, 3]
        ax.plot([prev_origin[0], origin[0]], [prev_origin[1], origin[1]], [prev_origin[2], origin[2]], 'k--')
# #打印各坐标系原点的在世界坐标系下的坐标值
print("打印基于机体坐标系Sigma_B的坐标系位置信息")
for i in range(CoordinatePoints_num):
    print(np.round(T[i][:3, 3], 5))

p=[]
print("打印基于机械臂坐标系Sigma_0的坐标系位置信息")
for i in range(CoordinatePoints_num-1):
    print(np.round(T_dh[i][:3, 3], 5))
    if i==3:
        p=(np.round(T_dh[i][:3, 3], 5))
# # 调用函数
print("角度解：")
theta= calculate_angles_all(p[0], p[1], p[2], l1, l2, l3) #初始角度为0，以所有坐标点在一个直线上
theta1=[]#初始角度为0，以DH表为基础
for i in range(len(theta)):
    theta1.append([theta[i][0],theta[i][1],theta[i][2]])
    theta1[i][1] = theta[i][1]+joints_angle_init[1]
    theta1[i][2] = theta[i][2]+joints_angle_init[2]
for i in range(len(theta)):
    print(f"第{i+1}组解: t1:{theta1[i][0]:.6f} t2:{theta1[i][1]:.6f} t3:{theta1[i][2]:.6f}")

print("位置验证：")
print("打印基于机械臂坐标系Sigma_0的坐标系位置信息")
for i in range(len(theta)):
    theta1 = theta[i][0]
    theta2 = theta[i][1]
    theta3 = theta[i][2]
    x=-math.cos(theta1)*(l2*math.sin(theta2)+l3*math.sin(theta2+theta3))
    y=-math.sin(theta1)*(l2*math.sin(theta2)+l3*math.sin(theta2+theta3))
    z=l1+l2*math.cos(theta2)+l3*math.cos(theta2+theta3)
    print(f"第{i+1}组解: x:{x:.6f} y:{y:.6f} z:{z:.6f}")
print("打印基于机械臂坐标系Sigma_B的坐标系位置信息")
for i in range(len(theta)):
    theta1 = theta[i][0]
    theta2 = theta[i][1]
    theta3 = theta[i][2]
    x=-math.cos(theta1)*(l2*math.sin(theta2)+l3*math.sin(theta2+theta3))
    y=-math.sin(theta1)*(l2*math.sin(theta2)+l3*math.sin(theta2+theta3))
    z=l1+l2*math.cos(theta2)+l3*math.cos(theta2+theta3)
    P_03 = np.array([x, y, z, 1])
    P_B3= np.dot(T_B0, P_03)
    print(f"第{i+1}组解: x:{P_B3[0]:.6f} y:{P_B3[1]:.6f} z:{P_B3[2]:.6f}")

    # # 已知末端点在机体Sigma_B坐标的位置，求末端点在Sigma_0的位置信息
    # T_B0_inv = np.linalg.inv(T_B0)
    # P_03_=np.dot(T_B0_inv, P_B3)
    # print(f"第------{i+1}组解: x:{P_03_[0]:.6f} y:{P_03_[1]:.6f} z:{P_03_[2]:.6f}")


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

set_axes_equal(ax)
plt.show()

