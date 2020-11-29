#!/usr/bin/env bash

set -e

echo "Current user: $(whoami)"
echo "Iniatiate Synergy HHKB environment"
echo "CPU ARCH: $(uname -m)"

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./installer_base.sh

mkdir -p /home/hhkb

chown -R hhkb:fairspace /home/hhkb
chown -R hhkb:fairspace /opt/ros

# Enable no-password sudo  
apt_get_update_and_install sudo sed
sed -i /etc/sudoers -re 's/^%sudo.*/%sudo ALL=(ALL:ALL) NOPASSWD: ALL/g'

apt-get clean && rm -rf /var/lib/apt/lists/