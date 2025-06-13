#!/bin/bash
cd "$(dirname "$0")"

# Install and enable the startracker service.
sudo cp startracker.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable startracker.service
sudo systemctl start startracker.service

# Install nginx service
sudo cp nginx.conf /etc/nginx/nginx.conf
sudo nginx -t
sudo systemctl enable nginx.service
sudo systemctl restart nginx.service

# Install shutdown service (don't enable this one)
sudo cp startracker-shutdown.service /etc/systemd/system
sudo cp startracker-shutdown-handler.sh /usr/local/bin
sudo chmod +x /usr/local/bin/startracker-shutdown-handler.sh
sudo systemctl daemon-reload
