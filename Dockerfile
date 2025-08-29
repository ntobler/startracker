FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV HOME=/root

# Update and install minimal build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ninja-build \
    cmake \
    pkg-config \
    git \
    python3 \
    python3-pip \
    libjpeg-dev \
    libtiff-dev \
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libdrm-dev \
    libexpat1-dev \
    python3-jinja2 \
    python3-ply \
    python3-yaml \
    libclang-14-dev \
    libcairo2-dev \
    libopencv-dev \
    curl \
    clang-14 \
    llvm-dev \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install meson

# Install Rust toolchain
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Build libcamera 0.3.0 from source
WORKDIR /opt
RUN git clone https://git.linuxtv.org/libcamera.git \
 && cd libcamera \
 && git checkout v0.3.0 \
 && meson setup build \
 && ninja -C build \
 && ninja -C build install

# Ensure clang-sys can find libclang
RUN ln -sf /lib/x86_64-linux-gnu/libclang-14.so /lib/x86_64-linux-gnu/libclang.so
ENV LIBCLANG_PATH=/usr/lib/llvm-14/lib
ENV LD_LIBRARY_PATH=/usr/lib/llvm-14/lib:$LD_LIBRARY_PATH

# Update library cache
RUN ldconfig

# Set up working directory for Rust project
WORKDIR /workspace
