#!/bin/bash

# Set the actual ethernet interface name here
ETH_NAME="enp86s0"

# Run the first five commands with a delay
sudo ip addr flush dev $ETH_NAME
sleep 2

ip addr show dev $ETH_NAME
sleep 2

sudo ip addr add 10.5.5.1/24 dev $ETH_NAME
sleep 2

sudo ip link set $ETH_NAME up
sleep 2

sudo ip addr show dev $ETH_NAME

# Wait for user input to run the final command
read -p "Press Enter to execute the final command..."

sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i $ETH_NAME --bind-dynamic



