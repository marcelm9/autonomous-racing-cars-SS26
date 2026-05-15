# installs vim and adds some aliases to .bashrc for easier use of colcon and ros2
# only run once
# note: specifically made for my macbook, not sure if somebody else needs this

yes | sudo apt install vim
echo """
alias src=\"source /opt/ros/jazzy/setup.bash && source /arc2026/ws/install/setup.bash\ && echo Built and sourced\"
alias cb=\"colcon build && src\"
""" > ~/.bashrc
pip install scikit-image --break-system-packages
echo -e "\nRestart this terminal to activate the aliases, or run 'source ~/.bashrc' to activate them immediately.\n"
