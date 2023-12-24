FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive

# Dependencies
RUN apt update && apt install -q -y --no-install-recommends \
    # system
    gnupg2 \
    wget \
    zsh \
    # libraries
    libeigen3-dev \
    # cpp
    build-essential \
    cmake \
    libgtest-dev \
    # python
    python3 \
    python3-dev \
    python3-pip \
    # documentation
    doxygen \
    graphviz \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Coverage
RUN python3 -m pip install gcovr

# Clang
RUN wget --quiet -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
    && echo "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-14 main\ndeb-src http://apt.llvm.org/focal/ llvm-toolchain-focal-14 main" > /etc/apt/sources.list.d/clang.list \
    && apt update \
    && apt install -q -y --no-install-recommends clang-format-14 clang-tidy-14 clang-tools-14 clang-14 libclang-common-14-dev clangd-14 lld-14 llvm-14 libc++-14-dev libc++abi-14-dev libomp-14-dev  \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-14 14 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-14 14 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
