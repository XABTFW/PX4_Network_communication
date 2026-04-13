#!/bin/sh
#
# DJ_MK ASMG-MD Servo Driver Test Script
#
# This script tests the DJ_MK servo driver functionality
#

echo "=========================================="
echo "DJ_MK Servo Driver Test"
echo "=========================================="

# Check if driver is already running
if dj_mk status 2>/dev/null; then
    echo "Driver is already running, stopping it first..."
    dj_mk stop
    sleep 1
fi

# Start the driver
echo ""
echo "Starting DJ_MK driver..."
dj_mk start

if [ $? -eq 0 ]; then
    echo "✓ Driver started successfully"
else
    echo "✗ Failed to start driver"
    exit 1
fi

sleep 2

# Check driver status
echo ""
echo "Checking driver status..."
dj_mk status

echo ""
echo "=========================================="
echo "Test completed"
echo "=========================================="
echo ""
echo "To manually control the servo, use:"
echo "  param set DJ_MK_ENABLE 1"
echo "  param set DJ_MK_SERVO_ID 1"
echo "  param set DJ_MK_SPEED 256"
echo ""
echo "To stop the driver:"
echo "  dj_mk stop"
