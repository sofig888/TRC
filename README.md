# Dual Boot Windows 11/10 & Ubuntu 22.04 LTS for ROS2 Humble

This guide provides a step-by-step process to dual boot Windows 11/10 with Ubuntu 22.04 LTS and then install ROS2 Humble. This setup is crucial for various robotics projects.

**Important Advice:** If you do not have an NVIDIA graphics card in your laptop or desktop, you may encounter problems controlling simulations and post-processing in different robotics projects.

## Requirements

* **Operating System:** Windows 11 or 10.
* **USB Drive:** 8GB+ USB drive to flash the Ubuntu distribution.
* **Free Space:** 50GB+ of free space on your SSD or HDD.

## Dual Boot Setup: Windows & Ubuntu

Follow the comprehensive guide in this video for the Windows configuration steps:
[How to Dual Boot Windows 11 and Ubuntu in 2025 (Step by Step)](https://youtu.be/MPMnizrPvHE?si=mFPrKJ1Zas_9DEb7) by Crown GEEK.

### Steps for Ubuntu Installation:

1.  **Download Ubuntu Desktop 22.04 LTS:**
    * Download the `.iso` file from the official releases page: [https://releases.ubuntu.com/22.04/](https://releases.ubuntu.com/22.04/?_ga=2.149898549.2084151835.1707729318-1126754318.1683186906&_gl=1*abt9uh*_gcl_au*MzY1MDI4NzAyLjE3NTU4MTE0NTQ.)
    * Ensure you download the **image desktop version**.

2.  **Install Rufus:**
    * Download Rufus from its official website: [https://rufus.ie/en/](https://rufus.ie/en/)

3.  **Prepare for Installation:**
    * **Disable Secure Boot:** Before proceeding with the hard steps, you need to disable Secure Boot from your PC's BIOS settings [00:00:33]. The video provides guidance on how to access and disable it [00:00:50].
    * **Create Partitions:** You will need to create a new partition for Ubuntu. The video demonstrates how to shrink your existing Windows partition to create unallocated space and then format it for Ubuntu [00:01:46].
    * **Create Bootable USB:** Use Rufus to create a bootable USB drive with the downloaded Ubuntu ISO [00:02:54].

4.  **Install Ubuntu:**
    * After connecting the bootable USB drive, configure it as a flash drive.
    * Boot from the USB drive and select your prepared partition to install Ubuntu [00:05:09].
    * Follow the on-screen instructions to complete the Ubuntu installation.

## ROS2 Humble Setup in Dual Boot Ubuntu

Once Ubuntu is successfully installed, follow this tutorial to set up ROS2:
[How to install ROS | Getting Ready to Build Robots with ROS #3](https://youtu.be/uWzOk0nkTcI?si=SQINbyTudW75mUYY) by Articulated Robotics.

If you are new to ROS, Articulated Robotics also offers an excellent introductory playlist to understand the basics of this robotics operating system:
[ROS Complete Guide Playlist](https://youtube.com/playlist?list=PLunhqkrRNRhYYCaSTVP-qJnyUPkTxJnBt&si=GXRbBpOZUJB1KvFU)

### Installing ROS2 Humble

The version of ROS2 to be installed is Humble. Refer to the official documentation for detailed instructions: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Open a terminal in Ubuntu (Ctrl + Alt + T) and follow these individual steps:

#### Step 1: Add Universe Repository and Install Prerequisites
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

#### Step 2: Install curl and Update Package List
```bash
sudo apt update && sudo apt install curl -y
```

#### Step 3: Add ROS2 GPG Key and Repository
Get the latest ROS2 apt source version:
```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
```

Download the ROS2 apt source package:
```bash
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
```

Install the ROS2 apt source:
```bash
sudo dpkg -i /tmp/ros2-apt-source.deb
```

#### Step 4: Update Package List and Upgrade System
```bash
sudo apt update
sudo apt upgrade
```

#### Step 5: Install ROS2 Humble Desktop
```bash
sudo apt install ros-humble-desktop
```

#### Step 6: Install Development Tools
```bash
sudo apt install ros-dev-tools
```

#### Step 7: Test ROS2 Installation
After completing the installation, test ROS2 with a simple talker-listener example to verify everything is working correctly.

First, source the ROS2 setup file:
```bash
source /opt/ros/humble/setup.bash
```

**Note:** Try some examples to verify your installation.

**Talker-Listener Demo:**
This is the most simple representation where different programming files like Python or C++ can communicate with each other to make more complex interactions for robotics projects.

**Terminal 1 - Run C++ Talker:**
Open a terminal and run:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 - Run Python Listener:**
Open a second terminal and run:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

If both nodes are running successfully, you should see the talker publishing messages and the listener receiving them. This confirms that ROS2 is properly installed and different programming languages can communicate through ROS2 topics.
