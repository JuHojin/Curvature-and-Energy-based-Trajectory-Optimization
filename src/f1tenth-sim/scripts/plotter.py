#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive
from control_msgs.msg import JointControllerState
import threading

class StateAndVelocityProfilePlotter:
    def __init__(self):
        rospy.init_node('StateAndVelocityProfilePlotter', anonymous=True)

        self.update_rate = 0.1
        self.current_time_step = 0.0
        self.time_window = 30.0

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
            'right_steering': 0.0,
            'average_steering': 0.0,
            'distances': [],
            'velocity_profile': [],
            'velocity_constraints_max_vel': [],
            'velocity_constraints_lat_acc': []
        }
        
        # Initialize historical data for plotting
        self.data = {
            'actual_velocity': [],
            'speed_command': [],
            'steering_command': [],
            'average_steering': [],
            'distances': [],
            'velocity_profile': [],
            'velocity_constraints_max_vel': [],
            'velocity_constraints_lat_acc': [],
            'time_steps': []
        }

        self.length_velocity_profile = 0

        # Subscribers
        rospy.Subscriber("/move_base/MyGlobalPlanner/plan/latest", Path, self.plan_callback)
        rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_profile", Float64MultiArray, self.profile_callback)
        rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_constraints", Float64MultiArray, self.constraints_callback)
        rospy.Subscriber("/car_1/base/odom", Odometry, self.velocity_callback)
        rospy.Subscriber('/car_1/command', AckermannDrive, self.command_callback)
        rospy.Subscriber("/car_1/left_steering_hinge_position_controller/state", JointControllerState, self.left_steering_state_callback)
        rospy.Subscriber("/car_1/right_steering_hinge_position_controller/state", JointControllerState, self.right_steering_state_callback)

        # Plot configurations with subplot index for each plot
        self.plot_configs = []
        
        if self.enable_velocity_plot:
            self.plot_configs.append({
                'title': 'Speed',
                'xlabel': 'Time [s]',
                'ylabel': 'Speed [m/s]',
                'legend': ['Actual Velocity', 'Speed Command'],
                'data_keys': ['actual_velocity', 'speed_command'],
                'subplot_index': 1
            })
        
        if self.enable_steering_plot:
            self.plot_configs.append({
                'title': 'Steering',
                'xlabel': 'Time [s]',
                'ylabel': 'Steering Angle [rad]',
                'legend': ['Actual Steering (Average)', 'Steering Command'],
                'data_keys': ['average_steering', 'steering_command'],
                'subplot_index': len(self.plot_configs) + 1
            })

        if self.enable_velocity_profile_plot:
            self.plot_configs.append({
                'title': 'Velocity Profile',
                'xlabel': 'Distance [m]',
                'ylabel': 'Speed [m/s]',
                'legend': ['Result', 'Max Vel', 'Lat Acc'],
                'data_keys': ['velocity_profile', 'velocity_constraints_max_vel', 'velocity_constraints_lat_acc'],
                'subplot_index': len(self.plot_configs) + 1
            })
        
    def plan_callback(self, msg):
        path = msg.poses
        num_points = len(path)
        distances = np.zeros(num_points)
        
        for i in range(1, num_points):
            pos1 = path[i - 1].pose.position
            pos2 = path[i].pose.position
            
            x1, y1 = pos1.x, pos1.y
            x2, y2 = pos2.x, pos2.y
            
            distances[i] = distances[i - 1] + np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
        self.latest_data['distances'] = distances.tolist()

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

    def right_steering_state_callback(self, msg):
        self.latest_data['right_steering'] = msg.process_value

    def update_data(self):
        rate = rospy.Rate(1 / self.update_rate)
        try:
            while not rospy.is_shutdown():
                # Update average steering value
                self.latest_data['average_steering'] = (self.latest_data['left_steering'] + self.latest_data['right_steering']) / 2

                # Append the latest data to historical lists for plotting
                for key in ['actual_velocity', 'speed_command', 'steering_command', 'average_steering']:
                    self.data[key].append(self.latest_data[key])
                    # Ensure data stays within the time window
                    if len(self.data[key]) * self.update_rate > self.time_window:
                        self.data[key] = self.data[key][-int(self.time_window / self.update_rate):]
                
                # Update velocity profile and constraints directly as lists
                self.data['velocity_profile'] = self.latest_data['velocity_profile']
                self.data['velocity_constraints_max_vel'] = self.latest_data['velocity_constraints_max_vel']
                self.data['velocity_constraints_lat_acc'] = self.latest_data['velocity_constraints_lat_acc']
                self.data['distances'] = self.latest_data['distances']

                # Update time_steps and ensure it stays within the time window
                self.data['time_steps'].append(self.current_time_step)
                if len(self.data['time_steps']) * self.update_rate > self.time_window:
                    self.data['time_steps'] = self.data['time_steps'][-int(self.time_window / self.update_rate):]
                
                self.current_time_step += self.update_rate
                
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

        if num_plots == 1:
            axs = [axs]

        # Connect the close event to the on_close callback
        def on_close(event):
            rospy.signal_shutdown("Plot window closed")
        fig.canvas.mpl_connect('close_event', on_close)

        while not rospy.is_shutdown():
            for config, ax in zip(self.plot_configs, axs):
                ax.clear()

                non_velocity_keys = []
                velocity_keys = []
                
                for key, label in zip(config['data_keys'], config['legend']):
                    if 'velocity_profile' in key or 'velocity_constraints' in key:
                        velocity_keys.append((key, label))
                    else:
                        non_velocity_keys.append((key, label))

                # Plot time-based data first
                for key, label in non_velocity_keys:
                    if key in self.data and self.data[key]:
                        ax.plot(self.data['time_steps'], self.data[key], label=label)

                # Plot distance-based data last
                for key, label in velocity_keys:
                    if key in self.data and self.data[key]:
                        ax.plot(self.data['distances'], self.data[key], label=label)

                # Set axis limits if applicable
                if config['data_keys'][0] in self.data and self.data[config['data_keys'][0]]:
                    if config['title'] == 'Velocity Profile' and self.data['velocity_constraints_max_vel']:
                        if self.data['velocity_constraints_max_vel']:
                            y_min = 0
                            y_max = max(self.data['velocity_constraints_max_vel']) * 1.2
                            ax.set_ylim(y_min, y_max)
                    else:
                        y_min = min(self.data[config['data_keys'][0]]) * 1.2
                        y_max = max(self.data[config['data_keys'][0]]) * 1.2
                        if y_min == y_max:
                            y_min = 0
                            y_max = 1
                        ax.set_ylim(y_min, y_max)
                else:
                    ax.set_ylim(-1, 1)

                ax.set_title(config['title'])
                ax.set_xlabel(config['xlabel'])
                ax.set_ylabel(config['ylabel'])
                ax.legend()
                ax.grid(True)

            plt.pause(1)
    
    def run(self):
        # Start a thread to consistently update data
        data_thread = threading.Thread(target=self.update_data)
        data_thread.start()

        # Run plotting in the main thread
        self.plot_data()

if __name__ == "__main__":
    plotter = StateAndVelocityProfilePlotter()
    plotter.run()
