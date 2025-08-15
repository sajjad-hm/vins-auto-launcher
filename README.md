
# VINS Auto Launcher

This ROS package automates the process of:

1. Launching **VINS-Mono** (`vins_estimator/android.launch`)
2. Starting an odometry recorder node or activity predictor node
3. Playing a specified ROS bag file
4. Automatically launch vins-mono and start prediction of the activity in real-time with pre built ML model.

Odometry or activity prediction data is saved or published as appropriate.

---


## **Prerequisites**

- ROS Melodic (tested on Ubuntu 18.04, Python 2.7)
- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) installed in your `catkin_ws/src`
- `rosbag` installed (comes with ROS by default)
- `joblib`, `numpy` (for predictor node, install via `pip` if needed)

---


## **Installation**

```bash
cd ~/catkin_ws/src
git clone https://github.com/sajjad-hm/vins-auto-launcher.git
cd ..
catkin_make
source devel/setup.bash
```

---


## **Usage**

### **Odometry Recording Launch Command**
Run the full process to record the raw_data feature (VINS-Mono → Bag playback → Feature Recorder):

```bash
roslaunch vins_auto_launcher full_pipeline.launch bag_file:=/absolute/path/to/your_file.bag
```

Example:
```bash
roslaunch vins_auto_launcher full_pipeline.launch bag_file:=/home/user/data/sample.bag
```

### **Activity Prediction Launch Command**
Run the full process with activity prediction (VINS-Mono → Bag playback → Activity Prediction):

```bash
roslaunch vins_auto_launcher full_pipeline_predict.launch bag_file:=/absolute/path/to/your_file.bag
```

Example:
```bash
roslaunch vins_auto_launcher full_pipeline_predict.launch bag_file:=/home/user/data/sample.bag
```

---


---

### **What Happens**
#### `full_pipeline.launch`
1. Launches VINS-Mono (`vins_estimator/android.launch`)
2. Waits **3 seconds**
3. Starts `vins_odom_recorder.py`:
   - Saves odometry from `/vins_estimator/odometry`
   - Stores CSV in the **same folder** as the `.bag` file
   - CSV filename: `<bag_name>_<timestamp>.csv`
4. Waits **3 seconds**
5. Plays the provided `.bag` file

#### `full_pipeline_predict.launch`
1. Launches VINS-Mono (`vins_estimator/android.launch`)
2. Waits **3 seconds**
3. Starts `predictor_node.py`:
   - Loads ML model and scaler from `models/`
   - Listens to `/vins_estimator/odometry`
   - Publishes activity prediction to `/activity_prediction`
4. Waits **6 seconds**
5. Plays the provided `.bag` file

---
#### `Monitor Activity Prediction in Real-Time`

```bash
rostopic echo /activity_prediction
```


### **CSV Format (Odometry Recorder)**
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
│   ├── full_pipeline.launch
│   └── full_pipeline_predict.launch
├── scripts/
│   ├── vins_odom_recorder.py
│   └── predictor_node.py
```

---


## **Tips**
- You must provide an **absolute path** for `bag_file` in the launch command.
- The package will **not overwrite** CSV files from previous runs — each has a unique timestamp.
- Ensure `vins_estimator` is built and **link the location of vins_eastimator in the launch file** and can run `android.launch` independently before using this package.
- For activity prediction, ensure the required model and scaler files are present in the `models/` directory.

---


## **License**
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
