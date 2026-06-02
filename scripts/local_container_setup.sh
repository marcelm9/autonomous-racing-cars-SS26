#!/bin/bash
# Installs vim and adds some aliases to .bashrc for easier use of colcon and ros2
# Run once. RUN WITHOUT SUDO.
# Note: Designed for a ROS 2 Jazzy environment (e.g., Ubuntu 24.04 VM/Docker on MacBook)

# Update package list first
sudo apt-get update

# Install apt dependencies
sudo apt-get install -y \
    vim \
    ros-jazzy-rmw-cyclonedds-cpp \
    lnav \
    ros-jazzy-tf-transformations \
    python3-skimage # Replacing pip install with apt package to avoid PEP 668 warnings

# Update .bashrc
cat << 'EOF' >> ~/.bashrc

# ROS 2 Jazzy custom prompt and aliases
PS1="\[\e[1;36m\]ros\[\e[0m\]@\[\e[1;32m\]\h\[\e[0m\]:\w\$ "
alias src="source /opt/ros/jazzy/setup.bash && source /arc2026/ws/install/setup.bash && echo 'Sourced'"

# Safe colcon build (always builds in the workspace root, then returns to current directory)
alias cb="cd /arc2026/ws && colcon build && src && echo 'Built and sourced' && cd -"
alias ros2="RCUTILS_COLORIZED_OUTPUT=1 PYTHONUNBUFFERED=1 ros2"

# alias foxglove="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# cyclonedds (added 'export' so it actually propagates to ROS 2)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

EOF

# Compile rangelibc (cloning to home directory instead of root /)
cd ~
git clone https://github.com/f1tenth/range_libc
cd range_libc/pywrapper
# Note: Ensure you have python-dev and cython installed for this to compile successfully
./compile.sh

# Setup VSCode syntax highlighting (using mkdir -p to avoid errors if it exists)
mkdir -p /arc2026/ws/.vscode
cat << 'EOF' > /arc2026/ws/.vscode/settings.json
{
    "python.analysis.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ],
    "python.autoComplete.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages"
    ]
}
EOF

# Done
echo -e "\n\nDone!\nRun 'source ~/.bashrc' to activate the aliases, or restart this terminal.\n\n"