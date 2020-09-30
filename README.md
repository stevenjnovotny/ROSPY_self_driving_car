This project was setup on a VM image using virtual box with ROS already installed. The following steps should help with setup.

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
if python errors occur, run the following

```bash
sudo easy_install pip==20.02
pip install testresources
pip install setuptools==44.0.0
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

if catkin_make fails, copy the following
```bash
dbw_mkz_msgs
```
to 
```bash
ros/src/
```

4. Run the udacity simulator (on the host machine using port forwarding)

