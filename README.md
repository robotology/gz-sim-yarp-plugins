# gz-yarp-plugins

**This repository contains a preliminary work in progress of integration of Modern Gazebo (gz) and YARP devices, a port of some functionalities of https://github.com/robotology/gazebo-yarp-plugins to Modern Gazebo. The repo is working in progress, and public interfaces can change without warning.**

## Install/Uninstall Gazebo Garden
#### Binary Installation in Ubuntu (or WSL)
First install some necessary tools:  
```
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```
Then install Gazebo Garden:
```
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```
#### Uninstalling binary install
```
sudo apt remove gz-garden && sudo apt autoremove
```

## Install YARP

#### Binary Installation in Ubuntu (or WSL)
```
sudo sh -c 'echo "deb http://www.icub.eu/ubuntu `lsb_release -cs` contrib/science" > /etc/apt/sources.list.d/icub.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 57A5ACB6110576A6
sudo apt update
sudo apt install yarp
```
#### Uninstalling binary install
```
sudo apt remove yarp && sudo apt autoremove
```

## Compilation
~~~
mkdir build
cd build
cmake ..
make
~~~
