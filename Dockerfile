FROM ros:kilted

# Set working directory
WORKDIR /workspace

# Install basic tools if needed
RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

# Default command
CMD ["/bin/bash"]
