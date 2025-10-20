FROM ros:jazzy-ros-base

# Use bash for RUN commands
SHELL ["/bin/bash", "-lc"]

# Tools & rosbridge
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

# rosdep bootstrap
RUN rosdep init || true
RUN rosdep update

COPY .dev/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]


WORKDIR /ws
