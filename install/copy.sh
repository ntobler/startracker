# Script to download python scripts to the raspberry pi.
# Assumes that a `~/.ssh/config` entry of the host `starpi` is available

ssh starpi "mkdir -p /home/pi/startracker && rm -rf /home/pi/startracker/* && mkdir -p /home/pi/startracker/camera"
scp ../startracker/camera_debug.py starpi:/home/pi/startracker
scp ../startracker/camera_stream.py starpi:/home/pi/startracker
scp ../startracker/camera_calibration.py starpi:/home/pi/startracker
scp ../startracker/camera_calibration.py starpi
scp -r '../startracker/camera/camera.py' starpi:'/home/pi/startracker/camera/camera.py'
scp -r '../startracker/camera/__init__.py' starpi:'/home/pi/startracker/camera/__init__.py'
scp -r '../startracker/camera/mock_camera.py' starpi:'/home/pi/startracker/camera/mock_camera.py'
scp ../startracker/image_processing.py starpi:/home/pi/startracker
# ssh starpi "cd /home/pi/startracker && python3 camera.py"
# scp starpi:/home/pi/startracker/test.npy ../test_images2/lightframe.npy
# scp starpi:/home/pi/startracker/test.jpg ../test_images2/test.jpg

