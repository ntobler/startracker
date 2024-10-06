# Raspberry Pi Setup

Make sure you use a `Raspberry Pi Zero 2 W`

On a debian linux host machine, install the Raspberry Pi Imager to flash fresh Raspberry Pi image.
Make sure you are using a new version of the Raspberry Pi Imager https://www.raspberrypi.com/software/
``` bash
sudo apt install rpi-imager
```

Choose `Raspberry Pi OS (other)` -> `Raspberry Pi OS Lite (64-bit)` and flash it on your SD Card.

> Tested with `Raspberry Pi OS Lite (64-bit)`, Release date: March 15th 2024, System: 64-bit, Kernel version: 6.6, Debian version: 12 (bookworm)

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
export DPKG_DEB_THREADS_MAX=1
sudo apt update && sudo apt upgrade -y
sudo bash -c "echo 'dtoverlay=arducam-pivariety' >> /boot/firmware/config.txt"
sudo bash -c "echo 'dtparam=i2c_arm=on' >> /boot/firmware/config.txt"
sudo bash -c "echo 'i2c-dev' >> /etc/modules"
```

Before we reboot, lets increase the swap
``` bash
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPFILE/#CONF_SWAPFILE/g' /etc/dphys-swapfile
sudo bash -c "echo 'CONF_SWAPSIZE=2048' >> /etc/dphys-swapfile"
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
sudo reboot now
```

The camera requires a specially built `libcamera` version. Install it with
``` bash
export DPKG_DEB_THREADS_MAX=1
mkdir temp && cd temp
wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
chmod +x install_pivariety_pkgs.sh
./install_pivariety_pkgs.sh -p libcamera_dev
sudo reboot now
```

You might want to test if the camera works using
```bash
libcamera-still -o test.jpg
```

Install rust toolchain. We set the number of cores to 1, as the Raspberry Pi runs low on RAM
and is more likely to access swap with the standard amount of cores 
```bash
mkdir -p ~/.cargo && echo -e "[build]\njobs = 1" >> ~/.cargo/config.toml
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install startracker git repo
``` bash
cd ~
export DPKG_DEB_THREADS_MAX=1
sudo apt install -y git libcairo2-dev python3-pip vim python3-virtualenv
git clone https://github.com/ntobler/startracker.git
virtualenv venv --system-site-packages
. venv/bin/activate
pip install -e ./startracker
sudo startracker/install/install_service.sh
```
Your Startracker is ready to go.
