#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from control_msgs.msg import JointControllerState
import threading

class StateAndVelocityProfilePlotter:
    def __init__(self):
        rospy.init_node('StateAndVelocityProfilePlotter', anonymous=True)

        # Parameters to enable or disable specific plots
        self.enable_velocity_plot = rospy.get_param('~enable_velocity_plot', True)
        self.enable_velocity_profile_plot = rospy.get_param('~enable_velocity_profile_plot', True)
        self.enable_steering_plot = rospy.get_param('~enable_steering_plot', True)

        # Initialize latest data values
        self.latest_data = {
            'actual_velocity': 0.0,
            'speed_command': 0.0,
            'steering_command': 0.0,
            'left_steering': 0.0,
            'velocity_profile': [],
            'velocity_constraints_max_vel': [],
            'velocity_constraints_lat_acc': []
        }
        
        # Initialize historical data for plotting
        self.data = {
            'actual_velocity': [],
            'speed_command': [],
            'steering_command': [],
            'left_steering': [],
            'velocity_profile': [],
            'velocity_constraints_max_vel': [],
            'velocity_constraints_lat_acc': []
        }

        self.length_velocity_profile = 0

        # Subscribers
        rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_profile", Float64MultiArray, self.profile_callback)
        rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_constraints", Float64MultiArray, self.constraints_callback)
        rospy.Subscriber("/car_1/base/odom", Odometry, self.velocity_callback)
        rospy.Subscriber('/car_1/command', AckermannDrive, self.command_callback)
        rospy.Subscriber("/car_1/left_steering_hinge_position_controller/state", JointControllerState, self.left_steering_state_callback)

        # Plot configurations with subplot index for each plot
        self.plot_configs = []
        
        if self.enable_velocity_plot:
            self.plot_configs.append({
                'title': 'Actual Velocity',
                'ylabel': 'Speed [m/s]',
                'legend': ['Actual Velocity', 'Speed Command'],
                'data_keys': ['actual_velocity', 'speed_command'],
                'subplot_index': 1
            })
        
        if self.enable_velocity_profile_plot:
            self.plot_configs.append({
                'title': 'Velocity Profiles',
                'ylabel': 'Speed [m/s]',
                'legend': ['Result', 'Max Vel', 'Lat Acc'],
                'data_keys': ['velocity_profile', 'velocity_constraints_max_vel', 'velocity_constraints_lat_acc'],
                'subplot_index': len(self.plot_configs) + 1
            })
        
        if self.enable_steering_plot:
            self.plot_configs.append({
                'title': 'Steering',
                'ylabel': 'Rad',
                'legend': ['Actual Left Steering', 'Steering Command'],
                'data_keys': ['left_steering', 'steering_command'],
                'subplot_index': len(self.plot_configs) + 1
            })

        # Thread for updating data consistently at a fixed rate
        self.update_rate = 0.1  # Update every 0.1 seconds

    def velocity_callback(self, msg):
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        self.latest_data['actual_velocity'] = (vel_x ** 2 + vel_y ** 2) ** 0.5

    def command_callback(self, msg):
        self.latest_data['speed_command'] = msg.speed
        self.latest_data['steering_command'] = msg.steering_angle

    def profile_callback(self, msg):
        self.latest_data['velocity_profile'] = msg.data
        self.length_velocity_profile = len(msg.data)

    def constraints_callback(self, msg):
        # Assuming the first part is max velocity and the second part is lateral acceleration
        half_length = len(msg.data) // 2
        self.latest_data['velocity_constraints_max_vel'] = msg.data[:half_length]
        self.latest_data['velocity_constraints_lat_acc'] = msg.data[half_length:half_length + self.length_velocity_profile]

    def left_steering_state_callback(self, msg):
        self.latest_data['left_steering'] = msg.process_value

    def update_data(self):
        rate = rospy.Rate(1 / self.update_rate)
        try:
            while not rospy.is_shutdown():
                # Append the latest data to historical lists for plotting
                for key in ['actual_velocity', 'speed_command', 'steering_command', 'left_steering']:
                    self.data[key].append(self.latest_data[key])

                # Update velocity profile and constraints directly as lists
                self.data['velocity_profile'] = self.latest_data['velocity_profile']
                self.data['velocity_constraints_max_vel'] = self.latest_data['velocity_constraints_max_vel']
                self.data['velocity_constraints_lat_acc'] = self.latest_data['velocity_constraints_lat_acc']

                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Shutdown Request: Stopping update thread.")

    def plot_data(self):
        num_plots = len(self.plot_configs)
        if num_plots == 0:
            rospy.logwarn("No plots are enabled. Please enable at least one plot to visualize data.")
            return

        plt.ion()
        fig, axs = plt.subplots(num_plots, 1, figsize=(10, 5 * num_plots))
        plt.show(block=False)

        # Ensure axs is always a list, even if only one plot is used
        if num_plots == 1:
            axs = [axs]

        while not rospy.is_shutdown():
            for config, ax in zip(self.plot_configs, axs):
                ax.clear()

                # Extract and plot data based on config
                for key, label in zip(config['data_keys'], config['legend']):
                    if key in self.data and self.data[key]:
                        ax.plot(self.data[key], label=label)

                # Set axis limits if applicable
                if config['data_keys'][0] in self.data and self.data[config['data_keys'][0]]:
                    if config['title'] == 'Velocity Profiles' and self.data['velocity_constraints_max_vel']:
                        # For velocity profiles plot, set ylim based on max velocity constraints
                        if self.data['velocity_constraints_max_vel']:
                            y_min = 0
                            y_max = max(self.data['velocity_constraints_max_vel']) * 1.2
                            ax.set_ylim(y_min, y_max)
                    else:
                        y_min = min(self.data[config['data_keys'][0]]) * 1.2
                        y_max = max(self.data[config['data_keys'][0]]) * 1.2
                        if y_min == y_max:
                            y_max = y_min + 1
                        ax.set_ylim(y_min, y_max)
                else:
                    ax.set_ylim(-1, 1)

                ax.set_title(config['title'])
                ax.set_xlabel('Time Step')
                ax.set_ylabel(config['ylabel'])
                ax.legend()
                ax.grid(True)

            plt.pause(0.1)

    def run(self):
        # Start a thread to consistently update data
        data_thread = threading.Thread(target=self.update_data)
        data_thread.start()

        # Run plotting in the main thread
        self.plot_data()

if __name__ == "__main__":
    plotter = StateAndVelocityProfilePlotter()
    plotter.run()
