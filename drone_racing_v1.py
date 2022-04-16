import argparse
import airsimdroneracinglab as airsim
import cv2
import threading
import time
import utils
import numpy as np
import math
from racer_class import Racer


# argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--race_track", type=str, choices=[	"Soccer_Field_Easy",
	"Soccer_Field_Medium",
	"ZhangJiaJie_Medium",
	"Building99_Hard",
	"Qualifier_Tier_1",
	"Qualifier_Tier_2",
	"Qualifier_Tier_3",
	"Final_Tier_1",
	"Final_Tier_2",
	"Final_Tier_3",
], default="Soccer_Field_Easy")
ap.add_argument("-v", "--enable_viz_traj", action="store_true", default=False)
ap.add_argument("-t", "--race_tier", type=int, choices=[1,2,3], default=1)
args = vars(ap.parse_args())

# initialize object of class racer
racer = Racer(drone_name="drone_1", viz_traj=True, viz_traj_color_rgba=[1.0, 0.0, 0.0, 1.0])
racer.load_level(args["race_track"])

if args["race_track"] == "Qualifier_Tier_1":
	args["race_tier"] = 1
if args["race_track"] == "Qualifier_Tier_2":
	args["race_tier"] = 2
if args["race_track"] == "Qualifier_Tier_3":
	args["race_tier"] = 3	

racer.start_race(args["race_tier"])

racer.initialize_drone()

racer.takeoff_with_moveOnSpline()

racer.get_ground_truth_gate_poses()

racer.start_image_callback_thread()

racer.start_odometry_callback_thread()

# move through spline, and finish the thread after function is over
racer.fly_through_all_gates_at_once_with_moveOnSpline().join()

# stop callback and reset race
racer.stop_image_callback_thread()
racer.stop_odometry_callback_thread()
racer.reset_race()