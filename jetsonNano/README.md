# This file describes the steps to configure the Jetson Nano board, from the OS to the ROS installation

## 1. Jetson configuration from the SD card image
**An image of the SD card with all the necessary configurations and the initial ROS nodes is available here : .....**

1. Flash the image on your SD Card (you can use etcher)
2. Insert the SD card in the Raspberry and Power ON the board (login = pi, password = geicar)
3. You will then have to change the ROS_ID at the end of the ~/.bashrc file in the docker container, with XX the number of the car (1, 2...): 
```sh
sudo docker start -ai ros-humble
sudo nano ~/.bashrc
```
Then change : 
```sh
export ROS_DOMAIN_ID = XX
```

4. Reboot :
```sh
sudo reboot
```


## 2. Jetson configuration from an empty SD card

## OS Installation
OS installation instructions for the Nvidia jetson board are available here: "https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit". Follow these instructions with :\
login = jetson\
hostname = geicar\
password = geicar

## Clone the project

```sh
git clone https://github.com/Fomys/geicar
```

## Automatic installation

1. Run the installation script:
```sh
sudo sh geicar/jetsonNano/install.sh
```

2. Run the configuration script. It is interactive and ask you many informations.
```sh
sudo sh geicar/jetsonNano/configure.sh
```

## Manual installation

All steps here are automated by a script found in `geicar/jetsonNano/install/[step].sh`

### [upgrade] General Updates and tools installation **(internet connection required)**

```sh
apt update
apt upgrade
apt install nano net-tools iputils-ping ufw ssh git
```

## [firewall] Firewall configuration to use ros2 :

1. Disable IPV6 in "/etc/default/ufw" : change IPV6=yes to IPV6=no 

2. Configure firewall rules to allow multicast :
```sh
sudo ufw allow from 192.168.1.1
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw disable
sudo ufw enable
```

3. Loopback multicast configuration :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
ifconfig lo multicast
```

4. Eth0 multicast configuration :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
ifconfig eth0 multicast
```

## [docker] Utilisation de docker pour ROS2

1. Connect the LIDAR to the Jetson board (by USB)

2. Install the latest version of docker (refer to [Install docker engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) to get the latest installation instructions)
```sh
sudo apt-get remove docker docker-engine docker.io containerd runc
sudo apt-get install ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
```

3. Create docker image
```sh
docker build geicar/jetsonNano/docker/Dockerfile
docker compose -f geicar/jetsonNano/docker/docker-compose.yml up -d
```


