from utils import AirSimSettingsCreator

AirSimSettingsCreator().write_airsim_neurips_baseline_settings_file()

# we run this so that airsim knows that there are two drones

# the settings.json has a tab called vehicles, and this have the settings for two drones:
# drone_1 and drone_2

# this settings have for example the pose of the drone in the beginning
# we can name the camera, set of the field of view of the camera

