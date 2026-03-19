#!/usr/bin/env bash
set -e

# Update package lists
sudo apt update

# Ensure we're on Debian
if [[ -f /etc/debian_version ]]; then
    debian_version=$(cut -d. -f1 /etc/debian_version)
    if [[ "$debian_version" -ne 13 ]]; then
        echo "This script is intended for Debian 13. Detected version: $debian_version" >&2
        exit 1
    fi
else
    echo "Not a Debian system." >&2
    exit 1
fi

# Install required packages
sudo apt install -y \
    g++ cmake \
    libeigen3-dev freeglut3-dev libopencv-dev \
    qt5-qmake qtbase5-dev qtdeclarative5-dev libqt5multimedia5 \
    protobuf-compiler libprotobuf-dev \
    libdc1394-25 libdc1394-dev \
    libv4l-0