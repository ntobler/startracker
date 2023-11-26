# Insall and enable the startracker service.
sudo cp startracker.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable startracker.service
sudo systemctl start startracker.service
