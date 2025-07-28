#!/bin/bash

# Script to run the complete visualization assembly pipeline
# Usage: ./run_vis_assembly.sh [task_name]
# Example: ./run_vis_assembly.sh guitar

set -e  # Exit on any error

# Disable exit on error for service waiting
set +e

# Default values
DEFAULT_TASK="guitar"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(realpath "$SCRIPT_DIR/../../../mr_planner/config/lego_tasks")"
SAVE_DIR="../outputs"
SIM_DELAY=50  # seconds to wait for simulation to start

# Get task name from command line argument or use default
TASK=${1:-$DEFAULT_TASK}

# Function to count bricks from JSON file
count_bricks_from_json() {
    local task_name=$1
    local json_file="$BASE_DIR/env_setup/env_setup_${task_name}.json"
    
    # Check if JSON file exists
    if [ ! -f "$json_file" ]; then
        echo "⚠ Warning: JSON file not found: $json_file"
        echo "Using default values: all brick counts = 0"
        num_b2=0; num_b3=0; num_b4=0; num_b5=0; num_b6=0; num_b9=0; num_b10=0; num_b12=0
        return
    fi
    
    echo "Parsing brick counts from: $json_file"
    
    # Count each brick type using grep and counting matches
    num_b2=$(grep -o '"b2_[0-9]*"' "$json_file" | wc -l)
    num_b3=$(grep -o '"b3_[0-9]*"' "$json_file" | wc -l)
    num_b4=$(grep -o '"b4_[0-9]*"' "$json_file" | wc -l)
    num_b5=$(grep -o '"b5_[0-9]*"' "$json_file" | wc -l)
    num_b6=$(grep -o '"b6_[0-9]*"' "$json_file" | wc -l)
    num_b9=$(grep -o '"b9_[0-9]*"' "$json_file" | wc -l)
    num_b10=$(grep -o '"b10_[0-9]*"' "$json_file" | wc -l)
    num_b12=$(grep -o '"b12_[0-9]*"' "$json_file" | wc -l)
    
    echo "Brick counts detected:"
    echo "  b2: $num_b2"
    echo "  b3: $num_b3" 
    echo "  b4: $num_b4"
    echo "  b5: $num_b5"
    echo "  b6: $num_b6"
    echo "  b9: $num_b9"
    echo "  b10: $num_b10"
    echo "  b12: $num_b12"
}

echo "============================================="
echo "Starting Visualization Assembly Pipeline"
echo "Task: $TASK"
echo "Base Dir: $BASE_DIR"
echo "Save Dir: $SAVE_DIR"
echo "============================================="

# Count bricks from JSON file
count_bricks_from_json "$TASK"

# Function to check if a ROS node is running
check_node_running() {
    local node_name=$1
    rosnode list | grep -q "$node_name"
}

# Function to wait for Gazebo to be fully ready
wait_for_gazebo_ready() {
    local timeout=${1:-60}
    local wait_time=0
    local check_interval=5
    
    echo "Waiting for Gazebo to be fully ready..."
    
    while [ $wait_time -lt $timeout ]; do
        # Check if essential Gazebo services are available
        if rosservice list | grep -q "/gazebo/get_model_state" && \
           rosservice list | grep -q "/gazebo/set_model_state" && \
           rostopic list | grep -q "/gazebo/model_states"; then
            echo "✓ Gazebo services and topics are available"
            return 0
        fi
        sleep $check_interval
        wait_time=$((wait_time + check_interval))
        echo "  Still waiting for Gazebo to be ready... (${wait_time}s elapsed)"
    done
    
    echo "✗ Timeout waiting for Gazebo to be ready"
    echo "Available Gazebo services:"
    rosservice list | grep gazebo | head -5
    return 1
}

# Function to wait for a ROS service
wait_for_service() {
    local service_name=$1
    local timeout=${2:-60}
    local wait_time=0
    local check_interval=2
    
    echo "Waiting for service: $service_name (timeout: ${timeout}s)"
    
    while [ $wait_time -lt $timeout ]; do
        if rosservice list | grep -q "$service_name"; then
            echo "✓ Service $service_name is available"
            return 0
        fi
        sleep $check_interval
        wait_time=$((wait_time + check_interval))
        echo "  Still waiting... (${wait_time}s elapsed)"
    done
    
    echo "✗ Timeout waiting for service $service_name"
    echo "Available services:"
    rosservice list | head -10
    return 1
}

# Function to kill background processes on exit
cleanup() {
    echo ""
    echo "Cleaning up background processes..."
    
    # Give processes a moment to finish what they're doing
    sleep 1
    
    if [ ! -z "$CAMERA_SERVER_PID" ]; then
        echo "Stopping camera_server.py (PID: $CAMERA_SERVER_PID)"
        kill -TERM $CAMERA_SERVER_PID 2>/dev/null || true
        sleep 2
        kill -KILL $CAMERA_SERVER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$DUAL_GP4_PID" ]; then
        echo "Stopping dual_gp4.launch (PID: $DUAL_GP4_PID)"
        kill -TERM $DUAL_GP4_PID 2>/dev/null || true
        sleep 2
        kill -KILL $DUAL_GP4_PID 2>/dev/null || true
    fi
    
    # Kill roslaunch processes
    echo "Killing roslaunch processes..."
    pkill -f "roslaunch.*dual_gp4" 2>/dev/null || true
    pkill -f "camera_server.py" 2>/dev/null || true
    
    # Kill Gazebo processes more aggressively
    echo "Killing Gazebo processes..."
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    pkill -f "gazebo" 2>/dev/null || true
    
    # Force kill any remaining processes with SIGKILL
    sleep 2
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "gazebo" 2>/dev/null || true
    pkill -9 -f "roslaunch.*dual_gp4" 2>/dev/null || true
    
    # Clean up any shared memory segments used by Gazebo
    echo "Cleaning up shared memory..."
    ipcs -m | grep $(whoami) | awk '{print $2}' | xargs -r ipcrm -m 2>/dev/null || true
    
    echo "Cleanup complete."
}

# Function to clean up any existing processes before starting
cleanup_existing() {
    echo "Checking for existing Gazebo processes..."
    if pgrep -f "gzserver\|gzclient\|gazebo" > /dev/null; then
        echo "Found existing Gazebo processes. Cleaning them up..."
        pkill -f "gzserver" 2>/dev/null || true
        pkill -f "gzclient" 2>/dev/null || true
        pkill -f "gazebo" 2>/dev/null || true
        sleep 3
        # Force kill if still running
        pkill -9 -f "gzserver" 2>/dev/null || true
        pkill -9 -f "gzclient" 2>/dev/null || true
        pkill -9 -f "gazebo" 2>/dev/null || true
        echo "✓ Existing Gazebo processes cleaned up"
    else
        echo "✓ No existing Gazebo processes found"
    fi
}

# Set up cleanup on script exit
trap cleanup EXIT

# Clean up any existing processes before starting
cleanup_existing

echo ""
echo "Step 1: Starting Gazebo simulation..."
echo "----------------------------------------"

# Start the simulation in background
roslaunch robot_digital_twin dual_gp4.launch num_b2:=$num_b2 num_b3:=$num_b3 num_b4:=$num_b4 num_b5:=$num_b5 num_b6:=$num_b6 num_b9:=$num_b9 num_b10:=$num_b10 num_b12:=$num_b12 &
DUAL_GP4_PID=$!

echo "Started dual_gp4.launch (PID: $DUAL_GP4_PID)"
echo "Waiting ${SIM_DELAY} seconds for simulation to fully initialize..."

# Wait for simulation to start
sleep $SIM_DELAY

# Check if Gazebo is running
if ! check_node_running "/gazebo"; then
    echo "✗ Gazebo node not found! Simulation may have failed to start."
    exit 1
fi

echo "✓ Gazebo node is running"

# Wait for Gazebo to be fully ready with services
echo "Checking if Gazebo is fully ready..."
if ! wait_for_gazebo_ready 30; then
    echo "⚠ Gazebo may not be fully ready, but continuing..."
else
    echo "✓ Gazebo is fully ready"
fi

echo ""
echo "Step 2: Starting camera server..."
echo "----------------------------------------"

# Start camera server in background
cd "$SCRIPT_DIR"
python3 camera_server.py &
CAMERA_SERVER_PID=$!

echo "Started camera_server.py (PID: $CAMERA_SERVER_PID)"
echo "Giving camera server time to initialize..."
sleep 3

echo "Waiting for camera service to be available..."

# Wait for the camera save service
if ! wait_for_service "/save_gazebo_images" 60; then
    echo "Camera service failed to start. Check if camera_server.py is running correctly."
    echo "Trying to continue anyway..."
fi

echo ""
echo "Step 3: Running visualization assembly sequence..."
echo "----------------------------------------"

# Run the visualization script in foreground so we can see output
echo "Running: python3 vis_assembly_seq.py --base_dir $BASE_DIR --save_dir $SAVE_DIR --task $TASK"
python3 vis_assembly_seq.py --base_dir "$BASE_DIR" --save_dir "$SAVE_DIR" --task "$TASK"

echo ""
echo "============================================="
echo "✓ Visualization assembly sequence completed!"
echo "✓ Images saved to: $SAVE_DIR/$TASK"
echo "============================================="

# Note: cleanup() will be called automatically when script exits
