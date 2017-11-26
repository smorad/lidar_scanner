#!/bin/bash

# Install script as a service, so it starts on raspberry pi boot

set -ex
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

cp scanner.service /etc/systemd/system/
systemctl start scanner
systemctl enable myfirst
systemctl stop scanner

echo "Successfully setup scanner, it will run a scan on boot"

