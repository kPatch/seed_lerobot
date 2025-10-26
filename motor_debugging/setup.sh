#!/bin/bash

# LeKiwi Motor Testing Setup Script
# This script sets up the Python environment and installs all dependencies

set -e  # Exit on error

echo "=========================================="
echo "LeKiwi Motor Testing Setup"
echo "=========================================="
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version

# Create virtual environment
echo ""
echo "Creating virtual environment..."
if [ -d "venv" ]; then
    echo "Virtual environment already exists. Skipping creation."
else
    python3 -m venv venv
    echo "✓ Virtual environment created"
fi

# Activate virtual environment
echo ""
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo ""
echo "Installing dependencies from requirements.txt..."
pip install -r requirements.txt

# Create necessary directories
echo ""
echo "Creating project directories..."
mkdir -p scripts
mkdir -p config
mkdir -p ros2_examples
mkdir -p logs
mkdir -p data

# Set permissions for scripts
echo ""
echo "Setting executable permissions for scripts..."
chmod +x scripts/*.py 2>/dev/null || true

# Check for serial port permissions
echo ""
echo "Checking serial port permissions..."
if groups | grep -q dialout; then
    echo "✓ User is in 'dialout' group (can access serial ports)"
else
    echo "⚠ User is NOT in 'dialout' group"
    echo "  To fix, run: sudo usermod -a -G dialout $USER"
    echo "  Then logout and login again"
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Activate virtual environment: source venv/bin/activate"
echo "2. Connect your LeKiwi mobile base via USB"
echo "3. Run port detection: python scripts/01_find_port.py"
echo "4. Follow the README.md for testing steps"
echo ""

