# VINS Auto Launcher

This ROS package automates the process of:

1. Launching **VINS-Mono** (`vins_estimator/android.launch`)
2. Starting an odometry recorder node
3. Playing a specified ROS bag file

The odometry data is saved into a CSV file whose name matches the bag file name, with a timestamp suffix to avoid overwriting.

---

## **Prerequisites**

- ROS Melodic (tested on Ubuntu 18.04, Python 2.7)
- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) installed in your `catkin_ws/src`
- `rosbag` installed (comes with ROS by default)

---

## **Installation**

```bash
cd ~/catkin_ws/src
git clone https://github.com/sajjad-hm/vins_auto_launcher.git
cd ..
catkin_make
source devel/setup.bash
```

---

## **Usage**

### **Basic Command**
Run the full pipeline (VINS → Recorder → Bag playback):

```bash
roslaunch vins_auto_launcher full_pipeline.launch bag_file:=/absolute/path/to/your_file.bag
```

Example:
```bash
roslaunch vins_auto_launcher full_pipeline.launch bag_file:=/home/user/data/sample.bag
```

---

### **What Happens**
1. Launches VINS-Mono (`vins_estimator/android.launch`)
2. Waits **3 seconds**
3. Starts `vins_odom_recorder.py`:
   - Saves odometry from `/vins_estimator/odometry`
   - Stores CSV in the **same folder** as the `.bag` file
   - CSV filename: `<bag_name>_<timestamp>.csv`
4. Waits **3 seconds**
5. Plays the provided `.bag` file

---

### **CSV Format**
The CSV file contains:

| Position_x | Position_y | Position_z | Orientation_x | Orientation_y | Orientation_z | Orientation_w |
|------------|------------|------------|---------------|---------------|---------------|---------------|

Example:
```
0.123, -0.456, 1.789, 0.0, 0.0, 0.707, 0.707
```

---

## **File Structure**

```
vins_auto_launcher/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── full_pipeline.launch
├── scripts/
│   └── vins_odom_recorder.py
```

---

## **Tips**
- You must provide an **absolute path** for `bag_file` in the launch command.
- The package will **not overwrite** CSV files from previous runs — each has a unique timestamp.
- Ensure `vins_estimator` is built and can run `android.launch` independently before using this package.

---

## **License**
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
