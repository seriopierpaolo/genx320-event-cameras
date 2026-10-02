FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        gdb \
        cmake \
        git \
        libopencv-dev \
        ca-certificates \
        python3 \
    && rm -rf /var/lib/apt/lists/*

# Build Google test
ARG GTEST_VERSION=v1.15.0
RUN cd $CODE_DIR && \
    git clone --depth 1 --branch $GTEST_VERSION https://github.com/google/googletest.git && \
    cd googletest && \
    mkdir build && cd build && \
    cmake .. && \
    make -j `nproc` install
