#stage 1, build and test
FROM ros:jazzy AS builder

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    ros-jazzy-message-filters \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . src/sensor_fusion_pkg/

#build the package
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select sensor_fusion_pkg

#run tests at build time, build fails if tests fail
RUN . /opt/ros/jazzy/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    pytest src/sensor_fusion_pkg/test/test_fused_data.py -v

#stage 2, Runtime
FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    ros-jazzy-message-filters \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ros2_ws/install /ros2_ws/install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "sensor_fusion_pkg", "fused_data.launch.py"]
