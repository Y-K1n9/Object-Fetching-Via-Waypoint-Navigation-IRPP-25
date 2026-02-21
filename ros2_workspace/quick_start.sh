#!/bin/bash
# quick_start.sh
# ONE-CLICK START script for Object Fetching Project
# Usage: ./quick_start.sh

cd "$(dirname "$0")"

echo "=========================================="
echo "   Object Fetching Project - Quick Start  "
echo "=========================================="

# 1. Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "❌ Error: Docker is not installed or not in PATH."
    exit 1
fi

# 2. Build the Docker image
echo "🔨 Building Docker image (object_fetcher:latest)..."
docker build -t object_fetcher:latest .

if [ $? -ne 0 ]; then
    echo "❌ Docker build failed."
    exit 1
fi

echo "✅ Docker build successful."

# 3. Run the container and launch the mission
echo "🚀 Launching Simulation..."
echo "   - Mounting workspace: $(pwd)"
echo "   - Connecting to X11 Display: $DISPLAY"

docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/home/ros/ws \
  object_fetcher:latest \
  /bin/bash -c "
    echo '🔧 Building ROS 2 workspace...' && \
    source /opt/ros/humble/setup.bash && \
    colcon build && \
    source install/setup.bash && \
    echo '✅ Build complete. Starting Main Launch...' && \
    ros2 launch object_fetcher full_nav2.launch.py
  "
