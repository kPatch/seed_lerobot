#!/bin/bash
# Robot Dog Voice Agent Launcher - Refactored Version (Console Mode)
# 
# This script launches the refactored LiveKit voice agent (main.py) in console mode,
# providing proper TTY context for interactive controls (Ctrl+B, Q to quit).
#
# The refactored version provides the same functionality as the original but with:
# - Better file organization (StateManager, DogCompanionAgent separated)
# - Cleaner architecture (programmatic tool registration)
# - Easier maintenance (modular structure)
#
# Usage:
#   ./run_main.sh

set -e  # Exit on any error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Path to the Python voice agent script (refactored version)
VOICE_AGENT_SCRIPT="$SCRIPT_DIR/main.py"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check prerequisites
check_prerequisites() {
    print_info "Checking prerequisites..."
    
    # Check if Python script exists
    if [ ! -f "$VOICE_AGENT_SCRIPT" ]; then
        print_error "Refactored voice agent script not found: $VOICE_AGENT_SCRIPT"
        print_error "Please ensure the package is built and the refactored version is available."
        exit 1
    fi
    
    # Check if Python3 is available
    if ! command -v python3 &> /dev/null; then
        print_error "python3 is not installed or not in PATH"
        exit 1
    fi
    
    # Check if we're in a proper terminal and determine mode
    if [ ! -t 0 ] || [ ! -t 1 ]; then
        print_warning "Not running in an interactive terminal"
        print_warning "Switching to start mode instead of console mode for compatibility"
        VOICE_AGENT_MODE="start"
    else
        print_success "Interactive terminal detected - using console mode"
        VOICE_AGENT_MODE="console"
    fi
    
    print_success "Prerequisites check passed"
}

# Function to setup environment
setup_environment() {
    print_info "Setting up environment..."
    
    # Set default environment variables if not already set
    export PYTHONUNBUFFERED=1  # Ensure immediate output
    
    # Load .env file if it exists in the script directory
    if [ -f "$SCRIPT_DIR/.env" ]; then
        print_info "Loading environment from $SCRIPT_DIR/.env"
        set -a  # Automatically export variables
        source "$SCRIPT_DIR/.env"
        set +a
    elif [ -f "$SCRIPT_DIR/../.env" ]; then
        print_info "Loading environment from $SCRIPT_DIR/../.env"
        set -a
        source "$SCRIPT_DIR/../.env"
        set +a
    else
        print_warning "No .env file found - using system environment variables"
    fi
    
    print_success "Environment setup complete"
}

# Function to display startup banner
show_banner() {
    echo
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          Robot Dog Voice Agent Launcher (Refactored)          ║"
    echo "║                        ${VOICE_AGENT_MODE^} Mode                            ║"
    echo "╠════════════════════════════════════════════════════════════════╣"
    echo "║  🏗️  Architecture: Modular (StateManager + DogCompanionAgent) ║"
    echo "║  🧰 Tools: Programmatic registration (cleaner separation)     ║"
    echo "║  📁 Organization: File-based modular structure               ║"
    echo "║                                                                ║"
    if [ "$VOICE_AGENT_MODE" = "console" ]; then
        echo "║  Interactive Controls:                                         ║"
        echo "║    [Ctrl+B] - Toggle between Text/Audio mode                  ║"
        echo "║    [Q]      - Quit the application                            ║"
        echo "║                                                                ║"
    else
        echo "║  Production Mode - Voice agent will run in background         ║"
        echo "║  Use Ctrl+C to stop the application                           ║"
        echo "║                                                                ║"
    fi
    echo "║  Wake Word: Say 'hey rufus' to activate voice interaction     ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo
}

# Function to handle cleanup on exit
cleanup() {
    print_info "Shutting down robot dog voice agent..."
    # Kill any background processes if needed
    # The Python script should handle its own cleanup
}

# Trap signals for graceful shutdown
trap cleanup EXIT INT TERM

# Main execution
main() {
    print_info "Starting Robot Dog Voice Agent (Refactored Version)..."
    
    # Run prerequisite checks
    check_prerequisites
    
    # Setup environment
    setup_environment
    
    # Show startup banner
    show_banner
    
    # Launch the voice agent in the appropriate mode
    print_info "Launching robot dog voice agent in $VOICE_AGENT_MODE mode..."
    print_info "Script: $VOICE_AGENT_SCRIPT"
    print_info "Mode: $VOICE_AGENT_MODE"
    print_info "Architecture: Modular (main.py + extracted classes)"
    echo
    
    # Execute the Python script with the determined mode
    # Use exec to replace the shell process, ensuring proper signal handling
    exec python3 "$VOICE_AGENT_SCRIPT" "$VOICE_AGENT_MODE"
}

# Run main function
main "$@" 