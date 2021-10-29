FROM registry.cn-hangzhou.aliyuncs.com/c137/ubuntu-dev

# install packages
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends libpcl-dev \
    && apt-get install -q -y --no-install-recommends \
    ros-noetic-tf ros-noetic-pcl-conversions \
    protobuf-compiler libprotobuf-dev \
    libceres-dev libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /buildspace && cd /buildspace \
    && git clone --depth 1 -b 20210324.2 https://github.com/abseil/abseil-cpp \
    && cd abseil-cpp && mkdir build && cd build \
    && cmake ..  -DCMAKE_BUILD_TYPE=Release -DABSL_ENABLE_INSTALL=ON \
    && make install -j \
    && cd / && rm -rf /buildspace/abseil-cpp


# setup entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["zsh"]

