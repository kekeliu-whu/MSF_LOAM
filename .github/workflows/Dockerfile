FROM registry.cn-hangzhou.aliyuncs.com/c137/ubuntu-dev

# install packages
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends libpcl-dev \
    && apt-get install -q -y --no-install-recommends \
    ros-noetic-tf ros-noetic-pcl-conversions \
    protobuf-compiler libprotobuf-dev \
    libceres-dev libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["zsh"]

