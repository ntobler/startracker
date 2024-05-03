#!/bin/bash
cd "$(dirname "$0")"

# Install and enable the startracker service.
sudo cp startracker.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable startracker.service
sudo systemctl start startracker.service

sudo cp nginx.conf /etc/nginx/sites-available/default
sudo systemctl enable nginx.service
sudo systemctl restart nginx.service
