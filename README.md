## Dependencies  

```
sudo apt install ros-jazzy-{depth-image-proc,pinocchio,camera-info-manager,realtime-tools}
cd src/yolo_ros
pip install -r requirements.txt
```

## Configuration (optional)

```
export CYCLONEDDS_URI=/home/snasn/data/cyclonedds.xml
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Detailed guide and overview  

A detailed guide and overview of the entire project is available as [mdbook](https://gianlucapandolfi.github.io/panda_ros/)
