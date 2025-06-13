#!/bin/bash

# Fail save check for /etc/noshutdown
if [ -f /etc/noshutdown ]; then
    echo "/etc/noshutdown exists, aborting shutdown."
    exit 0
fi

# Check that the startracker service is currently failed
state=$(systemctl is-failed startracker.service)
if [ "$state" != "failed" ]; then
    echo "startracker.service is not in 'failed' state, aborting shutdown."
    exit 0
fi

status=$(systemctl show -p ExecMainStatus --value startracker.service)
if [ "$status" -eq 31 ]; then
    echo "startracker.service exit code 31 received, triggering shutdown"
    shutdown -h now
fi
