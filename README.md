The Dnt package to run Motion Capture for multiple bots using aruco markers  

To run package:  

1) In your ros_ws/src
```git clone https://github.com/DNT-Dev/auv_mocap.git ```

```cd ..```  

```catkin_make ```  


Have camera feeds outputing to rostopics:  

/camera_1/image_raw  

/camera_2/image_raw  

/camera_3/image_raw  

... etc  


2) ```rosrun auv_mocap detector_node your_calibration_file.txt```

4) ```rosrun auv_mocap tf_node```
