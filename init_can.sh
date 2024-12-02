#!/bin/bash
sudo modprobe can
sudo modprobe can_raw

sudo ip link set down can2

sudo ip link set can2 type can bitrate 125000
sudo ip link set up can2
