from argparse import ArgumentParser
import airsimdroneracinglab as airsim
import cv2
import threading
import time
import utils
import numpy as np
import math

# Racer class
# drone name should be equal to the name in the settings file
# drone_name should match the name in ~/Document/AirSim/settings.json
class Racer(object):
	def __init__(self, drone_name="drone_1", viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0]):

		self.drone_name = drone_name
		self.gate_poses_ground_truth = None
		self.viz_traj = viz_traj
		self.viz_traj_color_rgba = viz_traj_color_rgba

		# abre conexão - give access to the drone functions
		self.airsim_client = airsim.MultirotorClient()
		self.airsim_client.confirmConnection()

		# MultirotorClient object is not thread safe
		# so to poll images and data to it we need to use other
		# multirotorclients()
		self.airsim_client_images = airsim.MultirotorClient()
		self.airsim_client_images.confirmConnection()
		self.airsim_client_odom = airsim.MultirotorClient()
		self.airsim_client_odom.confirmConnection()

		self.track_name = None

		# threads for odometry and images
		self.image_callback_thread = threading.Thread(
			target=self.repeat_timer_image_callback, args=(self.image_callback, 0.03)
		)
		self.odometry_callback_thread = threading.Thread(
			target=self.repeat_timer_odometry_callback,
			args=(self.odometry_callback, 0.02),
		)
		self.is_image_thread_active = False
		self.is_odometry_thread_active = False

		self.MAX_NUMBER_OF_GETOBJECTPOSE_TRIALS = (
			10  # see https://github.com/microsoft/AirSim-Drone-Racing-Lab/issues/38
		)


	# load the desired race track
	def load_level(self, track_name, sleep_sec=2.0):
		self.track_name = track_name
		self.airsim_client.simLoadLevel(self.track_name)
		self.airsim_client.confirmConnection()
		# let the envirnment load completely
		time.sleep(sleep_sec)

	def start_race(self, tier):
		self.airsim_client.simStartRace()

	def initialize_drone(self):
		# enable API access
		self.airsim_client.enableApiControl(vehicle_name = self.drone_name)
		# make sure the drone is armed
		self.airsim_client.arm(vehicle_name = self.drone_name)

		# set default gains for trajectory controller
		traj_tracker_gains = airsim.TrajectoryTrackerGains(
			kp_cross_track=5.0,
			kd_cross_track=0.0,
			kp_vel_cross_track=3.0,
			kd_vel_cross_track=0.0,
			kp_along_track=0.4,
			kd_along_track=0.0,
			kp_vel_along_track=0.04,
			kd_vel_along_track=0.0,
			kp_z_track=2.0,
			kd_z_track=0.0,
			kp_vel_z=0.4,
			kd_vel_z=0.0,
			kp_yaw=3.0,
			kd_yaw=0.1,
		)

		# Must be called once before either of the moveOnSpline*() APIs is called
		# this allows the user to set different values for the PID gains	
		self.airsim_client.setTrajectoryTrackerGains(traj_tracker_gains, vehicle_name=self.drone_name)
		time.sleep(0.2)

	def takeoff_with_moveOnSpline(self, takeoff_height=1.0):
		start_position = self.airsim_client.simGetVehiclePose(vehicle_name=self.drone_name).position

		# z axis: up is negative
		takeoff_waypoint = airsim.Vector3r(start_position.x_val, start_position.y_val, start_position.z_val - takeoff_height)

		self.airsim_client.moveOnSplineAsync([takeoff_waypoint], vel_max=15.0, acc_max=5.0,
			add_position_constraint=True, add_velocity_constraint=False, add_acceleration_constraint=False,
			viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, vehicle_name=self.drone_name,).join()

	def get_ground_truth_gate_poses(self):
		# get the names of all the objects which have gate on the name
		gate_names_sorted_bad = sorted(self.airsim_client.simListSceneObjects("Gate.*"))
		# format: GateN_GARBAGE
		# get the indexes
		gate_indices_bad = [int(gate_name.split("_")[0][4:]) for gate_name in gate_names_sorted_bad]

		gate_indices_correct = sorted(range(len(gate_indices_bad)), key= lambda k: gate_indices_bad[k])

		gate_names_sorted = [gate_names_sorted_bad[gate_idx] for gate_idx in gate_indices_correct]

		self.gate_poses_ground_truth = []

		# get the pose of every gate
		for gate_name in gate_names_sorted:
			curr_pose = self.airsim_client.simGetObjectPose(gate_name)
			self.gate_poses_ground_truth.append(curr_pose)

	def image_callback(self):
		# request imageType scene - RGB (this can be changed to depth, segmentation...)
		request = [airsim.ImageRequest("fpv_cam", airsim.ImageType.Scene, False, False)]
		response = self.airsim_client_images.simGetImages(request)
		img_rgb_1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8)
		img_rgb = img_rgb_1d.reshape(response[0].height, response[0].width, 3)

	def odometry_callback(self):
		# get information about the drone state
		drone_state = self.airsim_client_odom.getMultirotorState()
		# in world frame
		position = drone_state.kinematics_estimated.position
		orientation = drone_state.kinematics_estimated.orientation
		linear_velocity = drone_state.kinematics_estimated.linear_velocity
		angular_velocity = drone_state.kinematics_estimated.angular_velocity

	def start_image_callback_thread(self):
		if not self.is_image_thread_active:
			self.is_image_thread_active = True
			self.image_callback_thread.start()
			print("Started image callback thread")

	def stop_image_callback_thread(self):
		if self.is_image_thread_active:
			self.is_image_thread_active = False
			self.image_callback_thread.join()
			print("Stopped image callback thread.")			

	def start_odometry_callback_thread(self):
		if not self.is_odometry_thread_active:
			self.is_odometry_thread_active = True
			self.odometry_callback_thread.start()
			print("Started odometry callback thread")

	def stop_odometry_callback_thread(self):
		if self.is_odometry_thread_active:
			self.is_odometry_thread_active = False
			self.odometry_callback_thread.join()
			print("Stopped odometry callback thread.")

	# call task() method every "period" seconds.
	def repeat_timer_image_callback(self, task, period):
		while self.is_image_thread_active:
			task()
			time.sleep(period)

	def repeat_timer_odometry_callback(self, task, period):
		while self.is_odometry_thread_active:
			task()
			time.sleep(period)

	def fly_through_all_gates_at_once_with_moveOnSpline(self):
		if self.track_name in [
			"Soccer_Field_Medium",
			"Soccer_Field_Easy",
			"ZhangJiaJie_Medium",
			"Qualifier_Tier_1",
			"Qualifier_Tier_2",
			"Qualifier_Tier_3",
			"Final_Tier_1",
			"Final_Tier_2",
			"Final_Tier_3",
		]:
			vel_max = 30.0
			acc_max = 15.0

		if self.track_name == "Building99_Hard":
			vel_max = 4.0
			acc_max = 1.0

		return self.airsim_client.moveOnSplineAsync([gate_pose.position for gate_pose in self.gate_poses_ground_truth], 
			vel_max=vel_max, acc_max=acc_max, add_position_constraint=True, add_velocity_constraint=False, 
			add_acceleration_constraint=False, viz_traj=self.viz_traj, viz_traj_color_rgba=self.viz_traj_color_rgba, 
			vehicle_name=self.drone_name,)

	def reset_race(self):
		self.airsim_client.simResetRace()
