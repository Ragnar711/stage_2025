# Installation Raspberry pi for Drone

## Flash Sd card

-   Flash the sd card with Ubuntu 20.04
-   Connect the raspberry pi with mini hdmi/hdmi cable to a monitor
-   Connect a keyboard
-   Power on the rasperry pi

## Configure the Wifi

Set up: https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#4-boot-ubuntu-server

-   `sudo nano /etc/netplan/50-cloud-init.yaml`
-   Configure the Wifi with the name of the network and password
-   `sudo netplan apply`
-   `sudo reboot`

To know the ip of the robot: `ip a`

## Connect with ssh

-   `ssh ip_adress`
-   Connect with username and password
-   `sudo apt update`
-   `sudo apt upgrade`

## Install Docker

-   Set up: https://omar2cloud.github.io/rasp/rpidock/
-   `sudo usermod -a -G docker $USER`
-   reboot

## Copy Dockerfile

-   Create a folder _stage_2025_ in Documents: `mkdir stage_2025`
-   Open Folders with the GUI Application
    -   Other locations
    -   `sftp://ip_adress/`
-   Copy the Dockerfile on the folder _stage_2025_

## Build and Run the container

In the raspberry (via ssh):

-   `cd Documents\stage_2025`
-   `docker build --build-arg USER="gitlab-username" --build-arg TOKEN="your-personal-gitlab-token" -t "name_image"` .
-   `docker run -it --privileged --network="host" --name "name_container" "name_image"`
-   To run bash terminal of the container in an other tab terminal: `docker exec -it "name_container" bash`
