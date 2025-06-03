import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from std_msgs.msg import Float64
from uam_message.msg import arm_angle
# Set matplotlib backend
import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg'

class ArmVisualizer:
    def __init__(self):
        # Initialize ROS node first
        rospy.init_node('arm_visualizer', anonymous=True)
        
        # Arm parameters
        self.l1 = 0.053
        self.l2 = 0.21
        self.joint_num = 2
        self.CoordinatePoints_num = 2 + self.joint_num
        
        # DH parameters
        self.joints_alpha = [-math.pi/2, 0]
        self.joints_a = [0, 0.21]
        self.joints_d = [0.053, 0.007]
        self.joints_theta = [0, 0]
        self.joints_angle_init = [0, -math.pi/2]
        self.joints_angle = [0, 0]
        
        # Transformation matrices
        self.T_I = np.identity(4)
        self.T_B0 = np.array([
            [1, 0, 0, 0.104],
            [0, -1, 0, 0],
            [0, 0, -1, 0.10194],
            [0, 0, 0, 1]
        ])
        
        # Initialize plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Robot Arm Visualization')
        
        # Store plot objects for updating
        self.lines = []
        self.points = []
        
        # ROS subscribers
        rospy.Subscriber('/wjl/arm/real_angle', arm_angle, self.angle_cb)
        
        # Initial visualization
        self.update_visualization()

    def angle_cb(self, msg):
        self.joints_angle[0] = math.radians(msg.arm1_angle)
        self.joints_angle[1] = math.radians(msg.arm2_angle)
        math.radians
        self.update_visualization()

    def dh_matrix(self, alpha, a, d, theta):
        """Compute DH transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def update_visualization(self):
        """Update the arm visualization"""
        # Clear previous drawings
        for artist in self.ax.lines + self.ax.collections:
            artist.remove()
        
        # Compute transformations
        T = [self.T_I, self.T_B0]
        for i in range(self.joint_num):
            T.append(self.dh_matrix(
                self.joints_alpha[i],
                self.joints_a[i],
                self.joints_d[i],
                self.joints_theta[i] + self.joints_angle[i]
            ))
        
        # Multiply transformations
        for i in range(1, len(T)):
            T[i] = np.dot(T[i-1], T[i])
        
        # Plot arm
        positions = [t[:3, 3] for t in T]
        xs, ys, zs = zip(*positions)
        
        # Draw links
        self.ax.plot(xs, ys, zs, 'o-', color='blue', linewidth=2, markersize=8)
        
        # Draw coordinate frames
        for i, t in enumerate(T):
            origin = t[:3, 3]
            x_axis = origin + t[:3, 0] * 0.05
            y_axis = origin + t[:3, 1] * 0.05
            z_axis = origin + t[:3, 2] * 0.05
            
            self.ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r-')
            self.ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g-')
            self.ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b-')
        
        # Set equal aspect ratio
        self.set_axes_equal(self.ax)
        
        # Redraw
        plt.draw()
        plt.pause(0.01)

    @staticmethod
    def set_axes_equal(ax):
        """Set 3D axes to equal scale"""
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d()
        ])
        origin = np.mean(limits, axis=1)
        radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
        ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
        ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
        ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

    def run(self):
        """Main run loop"""
        try:
            plt.show(block=True)  # Use block=True for ROS compatibility
        except KeyboardInterrupt:
            rospy.signal_shutdown("Visualization closed")
            plt.close()

if __name__ == '__main__':
    try:
        visualizer = ArmVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass