# Use ROS2 Foxy as the base image
FROM ros:foxy

# Set the working directory
WORKDIR /wall_follower_ws

# Copy the source code
COPY . src/wall_follower_f1tenth

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Build the package
RUN /bin/bash -c '. /opt/ros/foxy/setup.bash; \
    colcon build --packages-select wall_follower_f1tenth'

# Set up the entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]

# Run the node
CMD ["ros2", "run", "wall_follower_f1tenth", "wall_follower_node"]
