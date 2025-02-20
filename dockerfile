# Use an official Ubuntu LTS as the base image
FROM ubuntu:20.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update package list and install essential tools and Eigen
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        nlohmann-json3-dev \
        clangd-10 \
        && ln -s /usr/bin/clangd-10 /usr/bin/clangd \
        && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /workspace

# Copy workspace vscode configs
COPY ./.vscode/ /workspace/.vscode/

# Default command when container starts
CMD [ "bash" ]
