#!/bin/bash

# Convenience script to run ROS2 Anywhere with proper user permissions
# This script automatically sets the user ID and group ID to match the host user

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

CURRENT_USER=$(whoami)
CURRENT_UID=$(id -u)
CURRENT_GID=$(id -g)

print_info "Detected user: $CURRENT_USER (UID: $CURRENT_UID, GID: $CURRENT_GID)"

if [ ! -d "./workspace" ]; then
    print_info "Creating workspace directory"
    mkdir -p ./workspace
fi

if [ ! -d "./.config" ]; then
    print_info "Creating config directory"
    mkdir -p ./.config
fi

export USER="$CURRENT_USER"
export CONTAINER_UID="$CURRENT_UID"
export CONTAINER_GID="$CURRENT_GID"

if [ -z "$VNC_PASSWORD" ]; then
    export VNC_PASSWORD="ros2vnc"
    print_info "Using default VNC password: $VNC_PASSWORD"
fi

# Parse command line arguments
COMMAND=${1:-up}
shift || true

case $COMMAND in
    up|start)
        print_info "Starting ROS2 Anywhere container..."
        docker compose up -d "$@"
        print_info "Container started successfully!"

        print_info "Waiting for container to be ready..."
        sleep 3

        print_info "Running post-setup to configure workspace..."
        docker exec -u "$CURRENT_USER" ros2_desktop_vnc post_setup.sh

        print_info "Setup complete!"
        print_info "Access the desktop at: http://localhost:80"
        print_info "VNC password: $VNC_PASSWORD"
        ;;
    down|stop)
        print_info "Stopping ROS2 Anywhere container..."
        docker compose down "$@"
        print_info "Container stopped successfully!"
        ;;
    restart)
        print_info "Restarting ROS2 Anywhere container..."
        docker compose down "$@"
        docker compose up -d "$@"

        print_info "Waiting for container to be ready..."
        sleep 3

        print_info "Running post-setup to configure workspace..."
        docker exec -u "$CURRENT_USER" ros2_desktop_vnc post_setup.sh

        print_info "Container restarted successfully!"
        print_info "Access the desktop at: http://localhost:80"
        ;;
    build)
        print_info "Building ROS2 Anywhere image..."
        docker compose build "$@"
        print_info "Build completed successfully!"
        ;;
    rebuild)
        print_info "Rebuilding ROS2 Anywhere image (no cache)..."
        docker compose build --no-cache "$@"
        print_info "Rebuild completed successfully!"
        ;;
    logs)
        print_info "Showing container logs..."
        docker compose logs -f "$@"
        ;;
    shell)
        print_info "Opening shell in container..."
        docker compose exec -u "$CURRENT_USER" ros2-desktop-vnc /bin/bash
        ;;
    status)
        print_info "Container status:"
        docker compose ps
        ;;
    clean)
        print_info "Cleaning up containers and images..."
        docker compose down --volumes --remove-orphans
        docker system prune -f
        print_info "Cleanup completed!"
        ;;
    setup)
        print_info "Running post-setup configuration..."
        docker exec -u "$CURRENT_USER" ros2_desktop_vnc post_setup.sh
        print_info "Post-setup completed!"
        ;;
    rebuild-workspace)
        print_info "Rebuilding workspace..."
        docker exec -u "$CURRENT_USER" ros2_desktop_vnc bash -c "cd ~/ros2_ws && rebuild_workspace.sh"
        print_info "Workspace rebuild completed!"
        ;;
    help|--help|-h)
        echo "ROS2 Anywhere - Convenience Script"
        echo ""
        echo "Usage: $0 [COMMAND] [OPTIONS]"
        echo ""
        echo "Commands:"
        echo "  up, start      Start the container (default)"
        echo "  down, stop     Stop the container"
        echo "  restart        Restart the container"
        echo "  build          Build the image"
        echo "  rebuild        Rebuild the image without cache"
        echo "  logs           Show container logs"
        echo "  shell          Open shell in running container"
        echo "  status         Show container status"
        echo "  setup          Run post-setup configuration"
        echo "  rebuild-workspace  Rebuild ROS2 workspace"
        echo "  clean          Clean up containers and images"
        echo "  help           Show this help message"
        echo ""
        echo "Environment Variables:"
        echo "  VNC_PASSWORD   Set VNC password (default: ros2vnc)"
        echo ""
        echo "Examples:"
        echo "  $0                    # Start container"
        echo "  $0 up                 # Start container"
        echo "  $0 down               # Stop container"
        echo "  $0 shell              # Open shell"
        echo "  $0 setup              # Run post-setup"
        echo "  $0 rebuild-workspace  # Rebuild workspace"
        echo "  VNC_PASSWORD=mypass $0 up  # Start with custom password"
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        print_info "Use '$0 help' for available commands"
        exit 1
        ;;
esac
