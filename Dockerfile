FROM ros:jazzy

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN userdel -r ubuntu

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo x11-apps \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip git "ros-$ROS_DISTRO-rviz2" "ros-$ROS_DISTRO-rqt*" "ros-$ROS_DISTRO-plotjuggler-ros"
ENV SHELL /bin/bash

WORKDIR /arc2026/
RUN git clone https://github.com/openai/gym -b 0.19.0 --depth 1
RUN cd gym && sed -i "/extras_require/d" setup.py && pip install -e . --break-system-packages
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && sed -i "/numpy/d" setup.py && pip install -e . --break-system-packages

WORKDIR /arc2026/ws/
# RUN pip install "setuptools==58.2.0"
RUN git clone https://github.com/CPS-TUWien/f1tenth_gym_ros src/f1tenth_gym_ros
RUN python3 -c 'import yaml;f=open("/arc2026/ws/src/f1tenth_gym_ros/config/sim.yaml");y=yaml.safe_load(f);y["bridge"]["ros__parameters"]["map_path"] = "/arc2026/maps/levine"; print(yaml.dump(y, default_flow_style=False, sort_keys=False))' > /arc2026/ws/src/f1tenth_gym_ros/config/sim.yaml.tmp \
            && mv /arc2026/ws/src/f1tenth_gym_ros/config/sim.yaml.tmp /arc2026/ws/src/f1tenth_gym_ros/config/sim.yaml
SHELL ["/bin/bash", "-c"]

RUN chown -R "$USER_UID:$USER_GID" /arc2026/

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

#RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
#
#RUN echo 'source $HOME/.cargo/env' >> $HOME/.bashrc
#
#RUN sudo apt update \
#    && sudo apt install -y git libclang-dev python3-pip python3-vcstool \
#    && PIP_BREAK_SYSTEM_PACKAGES=1 pip install git+https://github.com/colcon/colcon-cargo.git \
#    && PIP_BREAK_SYSTEM_PACKAGES=1 pip install git+https://github.com/colcon/colcon-ros-cargo.git
#
#RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust \
#    && pushd src/ros2_rust \
#    && git checkout 0fe1f21 \
#    && popd \
#    && vcs import src < src/ros2_rust/ros2_rust_jazzy.repos
#
#RUN git clone https://github.com/ros-drivers/ackermann_msgs.git src/deps/ackermann_msgs \
#    && cd src/deps/ackermann_msgs \
#    && git checkout ros2

ENTRYPOINT ["/bin/bash"]
CMD []

RUN . "/opt/ros/$ROS_DISTRO/setup.bash" \
        && PIP_BREAK_SYSTEM_PACKAGES=1 rosdep update --rosdistro "$ROS_DISTRO" \
        && PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -i --from-path src --rosdistro "$ROS_DISTRO" -y

