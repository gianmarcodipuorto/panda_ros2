# image_processing  

The *image_processing* package aims to smooth the output given by the YOLO model implementing a [**kalman filter**](https://en.wikipedia.org/wiki/Kalman_filter) and an [**heuristic**](https://en.wikipedia.org/wiki/Heuristic) to keep track of the tracked operator in the scene.  

## Nodes  

### *skeleton_tracker_yolo*  

The *skeleton_tracker_yolo* is the only node in the package. It depends on two messages coming from the network: 
- the **DetectionArray**, describing the keypoints of the operator in the scene, in terms of 3D components relative to a frame in the Tf tree. 
- the **Image** of the current scene. 

The **node's outputs** are: 
- the *Tfs* of the keypoints filtered
- a [**MarkerArray**](https://docs.ros.org/en/jade/api/visualization_msgs/html/msg/MarkerArray.html) message representing the operator's skeleton, that is, the union of the keypoints with segments in space.
- the **Image** of the current scene, containing the keypoints filtered by the node. 

The *synchronized_callback* function contains the main logic of the node, running everytime a pair of these messages are received using the [*message_filters*](https://docs.ros.org/en/rolling/p/message_filters/) package to synchronize the reception based on their timestamps.  
The function performs the following operations, in order: 
1. fill a map with the current [keypoints](./yolo_ros.md#yolo_ros) from the **DetectionArray** based on their unique id (1-17), excluding those outside the depth range of the camera. 
2. Update the [**decision metrics**](#decision-metrics) based on current and past keypoint confidence score. The metrics are relative to single keypoints and to the entire **tracking state** of the node.  
3. Based on the **decision metrics** the [**state** of the tracker](#tracking-states) is updated, along with the kalman filter states. 
4. The node's outputs are published to the networks. 

## Tracking states  

The node defines 3 states for the tracking process:

- **NO_PERSON**: there is no currently tracked operator in the scene. This is the initial state of the node. 
- **PERSON_TRACKED**: the node is tracking the operator inside the scene. The transition happens based on the moving average of the keypoints' moving average confidence score and the number of the current valid keypoints.
$$
conf_{MA} = \frac{\sum_i conf_{MA}^{Kp_i}}{m} \\\\
m = \\{ Kp_i \\: | \\: conf_{MA}^{Kp_i} \geq Kp_{conf\\: thresh} \\; \text{and} \\; conf^{Kp_i} \geq Kp_{conf \\: hallucination} \\} = \text{number of valid keypoints} \\\\
conf^{Kp_i} = \text{single keypoint current confidence score}\\\\
conf_{MA} \geq conf_{MA \\: thresh} \\; \text{and} \\; m \geq m_{thresh}
$$
- **PERSON_LOST**: the node is still tracking the operator but the total confidence is under the threshold. The node remains in this state for a certain amount of time before transitioning to the **NO_PERSON** state. If the confidence exceeds the threshold again the state will return to **PERSON_TRACKED**.


## Decision metrics  

- \\( Kp_{conf\\: thresh} \\): threshold relative to the single keypoint moving average confidence score.
- \\( Kp_{conf\\: hallucination} \\): threshold relative to the single keypoint hallucination confidence score; the threshold is used to avoid the keypoints that have a really low score in the total moving average calculation at each time step.
- \\( conf_{MA\\: thresh} \\): threshold relative to the total moving average confidence score; it is the primary index used for state transitions.
- \\( m_{thresh} \\): minimum number of valid keypoints required to transition to the **PERSON_TRACKED** state.
- \\( T_{lost} \\): maximum time the node can remain in the **PERSON_LOST** state.

## Kalman filter  

As stated in the introduction the node implements a kalman filter on the various keypoints of the skeleton. The filter handles various things:
- **sensor noise** derived from the depth sensor.
- **prediction of the keypoint position** in case of occlusions.
- **outlier removal** caused by strong noise fluctuations/occlusions that greatly change the depth value returned by the sensor. 

A **constant velocity model** is used for each keypoint. This assumption is obviously not true, so the covariance matrix of the process noise is used to characterize the acceleration of the keypoint in each direction, along the lines of a **uniformly accelerated motion**. Using this approach the process noise covariance matrix depends on the interval of time between 2 consecutive call of the filter's update function.  
The output noise is simply a diagonal matrix with the same covariance value for all directions.  

The removal of the outlier is based on the [mahalanobis distances](https://en.wikipedia.org/wiki/Mahalanobis_distance) of the innovation computed at each step; the distance is then compared to the quantile of a \\( \chi^2 \\) distribution with 3 degrees of freedom and with a confidence level of 0.9 (90%): if the distance is less than the quantile, the measure is accepted and the **update step** of the filter is computed; otherwise, it is rejected and the filter compute only the **prediction step**.
