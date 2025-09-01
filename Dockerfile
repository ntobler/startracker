# Dockerfile for building and testing startracker

# Use same system as Raspberry Pi OS (Debian Bookworm)
FROM debian:bookworm

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install all system-level dependencies in a single layer for efficiency
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ninja-build \
    cmake \
    pkg-config \
    git \
    python3-dev \
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
    gnupg \
    ca-certificates \
    libpython3.11-dev \
    meson \
    python3-poetry \
    wget
#  && rm -rf /var/lib/apt/lists/*

# Install system-wide Python tools
# RUN pip3 install meson poetry==2.1.3

# Install GitHub CLI
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | gpg --dearmor -o /usr/share/keyrings/githubcli-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
 && apt-get update && apt-get install -y gh

# Build libcamera 0.3.0 from source
WORKDIR /opt
RUN git clone https://git.linuxtv.org/libcamera.git \
 && cd libcamera \
 && git checkout v0.3.0 \
 && meson setup build \
 && ninja -C build \
 && ninja -C build install

# Set up environment for libclang and update library cache
ENV LIBCLANG_PATH=/usr/lib/llvm-14/lib
RUN ldconfig

# Install Rust toolchain as
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Set the final working directory for the CI job
WORKDIR /github/workspace
