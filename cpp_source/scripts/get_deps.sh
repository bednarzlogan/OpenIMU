#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  build-essential cmake ninja-build pkg-config \
  libeigen3-dev \
  nlohmann-json3-dev \
  libboost-dev        # <-- header-only bits incl. circular_buffer
