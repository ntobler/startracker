#!/bin/bash
status=$(systemctl show -p ExecMainStatus --value startracker.service)

if [ "$status" -eq 31 ]; then
    logger -t startracker "Exit code 31 received, triggering shutdown"
    sudo shutdown -h now
fi
