#!/usr/bin/env bash

set -euo pipefail

# Shared network initialization for the KUKA, Panda, and LUCID setup.
#
# Add interface names, IP addresses, routes, and any required sysctl rules
# here once the final network topology is confirmed.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export CYCLONEDDS_URI="file:///home/iwtros/Documents/ar/cyclonedds.xml"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
