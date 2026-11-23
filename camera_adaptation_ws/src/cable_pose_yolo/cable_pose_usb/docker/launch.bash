#!/usr/bin/env bash

clear

# Change to script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Default configuration
HEADLESS=false
BUILD=false
SERVICES=(
    cable_pose_yolo
)

# Additional containers based on flags
while [[ $# -gt 0 ]];
do
    case "$1" in 
        --headless)
            # Remove standard cable_pose_yolo and add headless
            SERVICES=("${SERVICES[@]/cable_pose_yolo}")
            SERVICES+=(cable_pose_yolo_headless)
            HEADLESS=true
            shift
            ;;
        --rs)
            SERVICES+=(realsense_setup)
            shift
            ;;
        --build)
            BUILD=true
            shift
            ;;
        *)
            echo "Unkown option: $1"
            exit 1
            ;;
    esac
done

if [ "$HEADLESS" != true ]; then
    xhost +local:
fi

# Rebuild everything from scratch
if [ "$BUILD" == true ];
then
    docker compose -f docker-compose.yaml build --no-cache "${SERVICES[@]}"
fi

docker compose -f docker-compose.yaml up --remove-orphans --force-recreate "${SERVICES[@]}"