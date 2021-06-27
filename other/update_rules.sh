sudo cp 10-local.rules /etc/udev/rules.d/10-local.rules
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
