#!/usr/bin/env bash

echo -e "Application for xarm5 teleoperation start...\n"

# Note: you should confirm the xarm5 ip which you're going to use for application
python ./scripts/xarm5_teleoperation.py 192.168.1.231

echo -e "Application for xarm5 teleoperation done...\n"
