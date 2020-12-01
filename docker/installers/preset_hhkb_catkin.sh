#!/usr/bin/env bash

set -e

echo "Current user: $(whoami)"
echo "Append ROS setup.bash to .bashrc"

cd ~

cat <<-EOF > "/home/hhkb/.bashrc"
if [[ -f devel/setup.bash ]]
then
    echo "Detect a catkin workspace at $(pwd)"
    source devel/setup.bash
else
    echo "Not found a catkin workspace yet, check /home/hhkb/catkin_ws?"
fi
EOF

