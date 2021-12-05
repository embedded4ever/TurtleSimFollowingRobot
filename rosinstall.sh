#!/bin/bash

check_lsb() {

    echo "Checking your current ubuntu version..."
    VERSION=$(lsb_release  -rs)

    if [[ $VERSION == "18.04" ]]
    then
        echo "->Ubuntu Version $VERSION"
    elif [[ $VERSION == "20.04" ]]
    then
        echo "-> Ubuntu Version $VERSION"
    else
        echo "-> Non-compatible version for installing ROS"
        exit 1
    fi
}

add_sourcelist() {   

    echo "New source list that will be added into this directory </etc/apt/source.list.d>"
    echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
    
}

key_added() {

    sudo apt install curl
    KEY=$(curl -sSL "http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654" | sudo apt-key add -)
    if [[ $KEY == "OK" ]]
    then 
        echo "-> Key is successfully added"
    else
        echo "-> Failed for key add"
        exit 1
    fi
}

get_repo() {

    sudo apt-get update && sudo apt-get upgrade
}

choice_packages() {

    read -n 1 -p "Would you like to install 
                1. ros-noetic-desktop-full (Recommend)
                2. ros-noetic-desktop
                3. ros-noetic-ros-base
                4. ros-noetic-ros-core 
                
                Please select (1/2/3/4) " ans;

    case $ans in
        1)
            echo "ros-noetic-desktop-full will be installed"
            sudo apt install ros-noetic-desktop-full;;
        2)
            echo "ros-noetic-desktop will be installed"
            sudo apt install ros-noetic-desktop;;
        3) 
            echo "ros-noetic-ros-base will be installed"
            sudo apt instsall ros-noetic-ros-base;;
        4)
            echo "ros-noetic-ros-core will be installed"
            sudo apt install ros-noetic-ros-core;;
        *)
            echo "Unknown"
            exit;;
    esac
}

setup_env() {

    echo "ROS environment set up now.."
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

verify_installation() {
    
    roscd
    DIR=$(/usr/bin/pwd)
    if [[ $DIR == "/opt/ros/noetic" ]]
    then
        echo "-> Installation Completed Succesfully"
    else
        echo "-> Installation failed"
    fi
}

check_lsb
add_sourcelist
key_added
get_repo
choice_packages
setup_env
verify_installation
