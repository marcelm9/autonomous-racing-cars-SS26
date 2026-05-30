# installs vim and adds some aliases to .bashrc for easier use of colcon and ros2
# only run once
# RUN WITHOUT SUDO
# note: specifically made for my macbook, not sure if somebody else needs this

# vim
yes | sudo apt install vim

# .bashrc
cat << 'EOF' >> ~/.bashrc
# ROS 2 Jazzy custom prompt and aliases
PS1="\[\e[1;36m\]ros\[\e[0m\]@\[\e[1;32m\]\h\[\e[0m\]:\w\$ "
alias src="source /opt/ros/jazzy/setup.bash && source /arc2026/ws/install/setup.bash && echo 'Sourced'"
alias cb="colcon build && src && echo 'Built and sourced'"
alias foxglove="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
EOF

# skimage
pip install scikit-image --break-system-packages

# tf-transformations
sudo apt-get update
yes | sudo apt-get install ros-jazzy-tf-transformations

# rangelibc
cd /
git clone https://github.com/f1tenth/range_libc
cd range_libc/pywrapper
./compile.sh

# # foxglove
# yes | sudo apt install ros-jazzy-foxglove-bridge

# syntax highlighting
mkdir /arc2026/ws/.vscode
cat << 'EOF' >> /arc2026/ws/.vscode/settings.json
{
    "python.analysis.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
    ],
    "python.autoComplete.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
    ]
}
EOF

# done
echo -e "\n\nDone\nRun 'source ~/.bashrc' to activate the aliases, or restart this terminal.\n\n"
