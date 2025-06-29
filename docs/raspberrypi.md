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
sudo systemctl disable dphys-swapfile
sudo dphys-swapfile swapon
sudo reboot now
```

The camera requires a specially built `libcamera` version. Install it with
``` bash
sudo dphys-swapfile swapon
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
sudo dphys-swapfile swapon
mkdir -p ~/.cargo && echo -e "[build]\njobs = 1" >> ~/.cargo/config.toml
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install startracker git repo
``` bash
sudo dphys-swapfile swapon
export DPKG_DEB_THREADS_MAX=1
sudo apt install -y git libcairo2-dev python3-pip vim python3-virtualenv nginx
cd ~
git clone https://github.com/ntobler/startracker.git
virtualenv venv --system-site-packages
. venv/bin/activate
pip install -e ./startracker
sudo startracker/install/install_service.sh
```

Set up automatic wifi switch over to a mobile hotspot (find with `sudo nmcli dev wifi`)
```bash
sudo nmcli dev wifi connect "wifi_name" password "password"
sudo nmcli connection modify wifi_name connection.autoconnect-priority 10
sudo nmcli connection modify preconfigured connection.autoconnect-priority 5
sudo nmcli connection modify wifi_name connection.autoconnect true
nmcli --fields autoconnect-priority,name connection
sudo systemctl restart NetworkManager
```

Disable unnecessary services that delay boot:
```bash
sudo systemctl mask ModemManager.service
sudo systemctl mask bluetooth.service
sudo systemctl mask avahi-daemon.service
sudo systemctl mask dphys-swapfile.service
sudo systemctl mask raspi-config.service
sudo systemctl mask e2scrub_reap.service
sudo systemctl mask alsa-restore.service
sudo systemctl mask alsa-state.service
sudo systemctl mask hciuart.service
sudo systemctl mask keyboard-setup.service
sudo systemctl mask triggerhappy.service
```

Limit journal size (logfile)
```bash
sudo sed -i '/^SystemMaxUse=/d' /etc/systemd/journald.conf
sudo sed -i '/^SystemKeepFree=/d' /etc/systemd/journald.conf
sudo sed -i '/^SystemMaxFileSize=/d' /etc/systemd/journald.conf
echo 'SystemMaxUse=10M' | sudo tee -a /etc/systemd/journald.conf
echo 'SystemKeepFree=5M' | sudo tee -a /etc/systemd/journald.conf
echo 'SystemMaxFileSize=5M' | sudo tee -a /etc/systemd/journald.conf
sudo systemctl restart systemd-journald
```

Optimize firmware config.txt
```ini
dtparam=audio=on -> dtparam=audio=off
camera_auto_detect=1 -> camera_auto_detect=0
display_auto_detect=1 -> display_auto_detect=0
dtoverlay=vc4-kms-v3d -> #dtoverlay=vc4-kms-v3d
[all]
dtoverlay=arducam-pivariety
hdmi_blanking=2
disable_splash=1
```

Disable some kernel modules:
/etc/modprobe.d/raspi-blacklist.conf
```ini
# Disable Bluetooth
blacklist btbcm
blacklist hci_uart

# Disable audio
blacklist snd_bcm2835
blacklist snd

# Disable GPU display stack (VC4 driver)
blacklist vc4

# Disable HDMI CEC and other HDMI modules
blacklist cec
blacklist drm
blacklist drm_kms_helper
```

Your Startracker is ready to go.
