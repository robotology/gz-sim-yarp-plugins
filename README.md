# gz-yarp-plugins

**This repository contains a preliminary work in progress of integration of Modern Gazebo (gz) and YARP devices, a port of some functionalities of https://github.com/robotology/gazebo-yarp-plugins to Modern Gazebo. The repo is working in progress, and public interfaces can change without warning.**

## Quick start
- [Install Gazebo Garden](#install_gazebo)
- [Install YARP](#install_yarp)
- [Install gz-sim-yarp-plugins](#install_gz-sim-yarp-plugins)
  

### <a name="install_gazebo"></a> Install Gazebo Garden
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

### <a name="install_yarp"></a> Install YARP

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

## <a name="install_gz-sim-yarp-plugins"></a> Install gz-sim-yarp-plugins
~~~
mkdir build
cd build
cmake ..
make
sudo make install
~~~
To notify Gazebo of the new plugins compiled, it is necessary to modify the GZ_SIM_SYSTEM_PLUGIN_PATH environment variable:
```
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:/path/to/the/install/folder/lib
```
where `/path/to/the/install/folder/lib` is the directory containing the `libgz-sim-yarp-forcetorque-system.so`, `libgz-sim-yarp-camera-system.so`... files (by default `/usr/local/lib`)

To avoid having to modify this environment variable each time, you can place this command in the `.bashrc` file in your home directory.
