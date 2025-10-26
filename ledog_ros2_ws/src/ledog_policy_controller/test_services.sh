#!/bin/bash
###############################################################################
# LeRobot Policy Controller - Service Test Script
#
# This script tests all three services provided by the policy controller.
# Run this after starting the policy_controller_node.
#
# Usage:
#   chmod +x test_services.sh
#   ./test_services.sh
#
# Author: kpatch
# License: MIT
###############################################################################

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   LeRobot Policy Controller - Service Test Script         ║${NC}"
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo ""

# Check if node is running
echo -e "${YELLOW}[1/5] Checking if policy_controller_node is running...${NC}"
if ros2 node list | grep -q "policy_controller_node"; then
    echo -e "${GREEN}✓ Node is running${NC}"
else
    echo -e "${RED}✗ Node is NOT running!${NC}"
    echo -e "${YELLOW}Start the node first with:${NC}"
    echo "  ros2 launch ledog_policy_controller policy_controller.launch.py"
    exit 1
fi
echo ""

# Test 1: Check initial status
echo -e "${YELLOW}[2/5] Testing get_policy_status service (initial state)...${NC}"
STATUS_OUTPUT=$(ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger 2>&1)
if echo "$STATUS_OUTPUT" | grep -q "success: true"; then
    echo -e "${GREEN}✓ Status service working${NC}"
    echo -e "  Response: $(echo "$STATUS_OUTPUT" | grep "message:" | cut -d"'" -f2)"
else
    echo -e "${RED}✗ Status service failed${NC}"
    echo "$STATUS_OUTPUT"
fi
echo ""

# Test 2: Start the policy
echo -e "${YELLOW}[3/5] Testing start_policy service...${NC}"
echo -e "${BLUE}Note: This will try to start the LeRobot process${NC}"
read -p "Press Enter to continue or Ctrl+C to cancel..."
START_OUTPUT=$(ros2 service call /ledog/start_policy std_srvs/srv/Trigger 2>&1)
if echo "$START_OUTPUT" | grep -q "success: true"; then
    echo -e "${GREEN}✓ Policy started successfully${NC}"
    echo -e "  Response: $(echo "$START_OUTPUT" | grep "message:" | cut -d"'" -f2)"
elif echo "$START_OUTPUT" | grep -q "already running"; then
    echo -e "${YELLOW}⚠ Policy is already running${NC}"
else
    echo -e "${RED}✗ Failed to start policy${NC}"
    echo -e "  Response: $(echo "$START_OUTPUT" | grep "message:" | cut -d"'" -f2)"
fi
echo ""

# Test 3: Check status while running
echo -e "${YELLOW}[4/5] Checking status after start...${NC}"
sleep 1
STATUS_OUTPUT=$(ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger 2>&1)
if echo "$STATUS_OUTPUT" | grep -q "running"; then
    echo -e "${GREEN}✓ Policy is running${NC}"
    echo -e "  Response: $(echo "$STATUS_OUTPUT" | grep "message:" | cut -d"'" -f2)"
else
    echo -e "${YELLOW}⚠ Policy may have exited${NC}"
    echo -e "  Response: $(echo "$STATUS_OUTPUT" | grep "message:" | cut -d"'" -f2)"
fi
echo ""

# Test 4: Stop the policy
echo -e "${YELLOW}[5/5] Testing stop_policy service...${NC}"
STOP_OUTPUT=$(ros2 service call /ledog/stop_policy std_srvs/srv/Trigger 2>&1)
if echo "$STOP_OUTPUT" | grep -q "success: true"; then
    echo -e "${GREEN}✓ Policy stopped successfully${NC}"
    echo -e "  Response: $(echo "$STOP_OUTPUT" | grep "message:" | cut -d"'" -f2)"
elif echo "$STOP_OUTPUT" | grep -q "already stopped"; then
    echo -e "${YELLOW}⚠ Policy was already stopped${NC}"
    echo -e "  Response: $(echo "$STOP_OUTPUT" | grep "message:" | cut -d"'" -f2)"
else
    echo -e "${RED}✗ Failed to stop policy${NC}"
    echo -e "  Response: $(echo "$STOP_OUTPUT" | grep "message:" | cut -d"'" -f2)"
fi
echo ""

# Final status check
echo -e "${YELLOW}Final status check...${NC}"
STATUS_OUTPUT=$(ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger 2>&1)
echo -e "  Response: $(echo "$STATUS_OUTPUT" | grep "message:" | cut -d"'" -f2)"
echo ""

# Summary
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Test Complete                                            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${GREEN}All three services were tested:${NC}"
echo "  • /ledog/start_policy"
echo "  • /ledog/stop_policy"
echo "  • /ledog/get_policy_status"
echo ""
echo -e "${YELLOW}Check the node logs for detailed output:${NC}"
echo "  ros2 topic echo /rosout"
echo ""

