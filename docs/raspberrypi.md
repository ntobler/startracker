# Raspberry Pi Setup

Make sure you use a `Raspberry Pi Zero 2 W`

On a debian linux host machine, install the Raspberry Pi Imager to flash fresh Raspberry Pi image.
``` bash
sudo apt install rpi-imager
```

Choose `Raspberry Pi OS (other)` -> `Raspberry Pi OS Lite (64-bit)` and flash it on your SD Card.

Under Advanced options enter following settings:
- Set hostname `startrackerpi`
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
ssh-copy-id -i ~/.ssh/id_rsa.pub pi@startrackerpi
```

SSH to the Pi with `ssh pi@startrackerpi` and setup with following commands:
``` bash
sudo apt install -y python3-pyqt5 python3-opengl
sudo apt update
sudo apt upgrade
sudo apt install -y python3-picamera2 --no-install-recommends
sudo apt install python3-pip
cd ~
git clone ssh://git@github.com:ntobler/startracker.git
pip install startracker
sudo startracker/install_service.sh
```
Your Startracker is ready to go.
