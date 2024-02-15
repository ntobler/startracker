# Raspberry Pi Setup

Make sure you use a `Raspberry Pi Zero 2 W`

On a debian linux host machine, install the Raspberry Pi Imager to flash fresh Raspberry Pi image.
Make sure you are using a new version of the Raspberry Pi Imager https://www.raspberrypi.com/software/
``` bash
sudo apt install rpi-imager
```

Choose `Raspberry Pi OS (other)` -> `Raspberry Pi OS Lite (64-bit) Legacy` and flash it on your SD Card.
> Make sure to use the legacy `Bullseye` Raspberry Pi OS, as it is required for the camera driver to work
  Check the Raspberry Pi Linux kernel version >6.1.21, as needed for the `Pivariety` dirver.
  Check with `hostnamectl`

Under Advanced options enter following settings:
- Set hostname `starpi`
- Enable SSH
  - Use password authentication
- Set username and password
  - Username: `pi`
  - Password: `strongpassword`
- Configure wireless LAN
  - Enter your WiFi settings


If you haven't already, create a private/public key pair for password-less authentication:
``` bash
cd ~/.ssh
ssh-keygen -b 2048 -t rsa
```
Register your public key to the Pi:
``` bash
ssh-copy-id -i ~/.ssh/id_rsa.pub pi@starpi
```

SSH to the Pi with `ssh pi@starpi` and setup with following commands:

Update system and enable the `Pivariety` kernel driver. Requires a reboot.
```bash
sudo apt update && sudo apt upgrade -y
sudo bash -c "echo 'dtoverlay=arducam-pivariety' >> /boot/config.txt"
sudo reboot now
```

The camera requires a specially built `libcamera` version. Install it with
``` bash
mkdir temp && cd temp
wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
chmod +x install_pivariety_pkgs.sh
./install_pivariety_pkgs.sh -p libcamera_dev
./install_pivariety_pkgs.sh -p libcamera_apps
```

Install Picamera2 dependencies
``` bash
sudo apt install -y python3-kms++
sudo apt install -y python3-pyqt5 python3-prctl libatlas-base-dev ffmpeg python3-pip
sudo pip3 install numpy --upgrade
sudo pip3 install picamera2 --upgrade
```

Install startracker git repo
``` bash
cd ~
sudo apt install -y git libcairo2-dev
git clone https://github.com/ntobler/startracker.git
pip install ./startracker
sudo startracker/install_service.sh
```
Your Startracker is ready to go.
