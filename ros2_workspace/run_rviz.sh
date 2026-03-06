#!/bin/bash
# run_rviz.sh — Launch RViz for visualization
# Usage: ./run_rviz.sh  (works from any directory)

cd "$(dirname "$0")"

CONTAINER_ID=$(docker ps -q --filter ancestor=object_fetcher)

if [ -z "$CONTAINER_ID" ]; then
    echo "❌ Error: Container not running. Run ./quick_start.sh first."
    exit 1
fi

echo "🚀 Launching RViz in container $CONTAINER_ID..."

docker exec -it $CONTAINER_ID bash -c "
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    rviz2 -d install/object_fetcher/share/object_fetcher/config/nav2_view.rviz
"
