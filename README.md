# study-gazebo-garden-yarp-plugins

## Install/Uninstall Gazebo Garden
#### Binary Installation in Ubuntu
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
## Install/Uninstall Gazebo Classic
Installation in Ubuntu with dependencies provided by conda-forge:
```
mamba install -c conda-forge gazebo
```
To uninstall:
```
mamba remove gazebo
```
