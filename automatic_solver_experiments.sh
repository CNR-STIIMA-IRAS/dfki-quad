#!/bin/bash

# Check if at least one branch and corresponding number is provided
echo "Usage: $0 <platform_name> <test_preset> <ros_domain_id> <branch1> <number1> <branch2> <number2> ..."


platform_name="$1"
test_preset="$2"
domain_id="$3"
shift 3
# Process arguments as pairs (branch, number)
while [ "$#" -gt 0 ]; do
    branch="$1"
    number="$2"
    shift 2  # Shift past the current branch and number

    echo "Switching to branch: $branch"

    # Ensure we are in a git repository
    if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
        echo "Error: This script must be run inside a Git repository."
        exit 1
    fi

    # Fetch latest changes and switch to the branch
    git fetch origin
    if ! git checkout "$branch"; then
        echo "Error: Could not switch to branch $branch"
        exit 1
    fi

    echo "Starting Docker container..."

    docker_options=(-t \
        --label="dfki_quad" \
        --net="host" \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${PWD}/ws/src:/root/ros2_ws/src" \
        --volume="/home/$USER/.ros_docker_bash_history:/root/.bash_history" \
	--device="/dev/input:/dev/input" \
	--device="/dev/ttyACM0:/dev/ttyACM0" \
	--privileged)

    # when on arm, try to mount the external ssd for data logging
    if [[ $(uname -i) = "aarch64" ]]; then
	docker_options+=(--mount type=bind,src=/media/external-drive,dst=/media/external-drive)
	docker_options+=(--mount type=bind,src=/usr/bin/tegrastats,dst=/usr/bin/tegrastats)
    elif [[ $(uname -i ) = "x86_64" ]]; then
	docker_options+=(--mount type=bind,source=/sys/class/powercap,target=/sys/class/powercap)
    fi

    # check, if a container with dfki_quad label is already there
    container_id=$(docker ps -aq --filter "label=dfki_quad")
    xhost +local:
    mkdir ${PWD}/ws/src/solver_experiments/results
    docker run  "${docker_options[@]}" dfki_quad:latest /bin/bash -ci "cbg_onboard && sr && export ROS_DOMAIN_ID=$domain_id && ros2 run solver_experiments solver_experiments --ros-args -p mpc_prediction_horizon:="$number" -p test_preset:="$test_preset""

    # Check if the container started successfully
    if [ $? -ne 0 ]; then
        echo "Error: Docker container failed to run."
        exit 1
    fi

    echo "Stopping container..."
    docker stop $(docker ps -q -f ancestor=dfki_quad:latest)

    echo "moving data"
    mv ${PWD}/ws/src/solver_experiments/results ${PWD}/ws/src/solver_experiments/${platform_name}-N${number}-${test_preset}_$(date '+%Y%m%d_%H%M')

    echo "Experiment on branch $branch with horizon $number completed."
done

echo "All experiments finished!"
