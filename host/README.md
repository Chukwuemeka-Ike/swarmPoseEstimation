# Host
This folder contains packages developed to run on the host computer.

## Swarm Centroid Estimator
The Swarm Centroid Estimator package contains two nodes for running and testing the overall estimation functionality. It also holds the 

### Centroid Calculator
This node serves an error-checking purpose. It gets complete information about all the Kheperas by subscribing to the */khepera_pose* ROS topic, then uses that to calculate the swarm's centroid. It then publishes the calculated centroid to the */true_centroid* topic.

### Centroid Estimator
This node will implement the same functionality as the Khepera centroid estimator package on the host machine, effectively making the host a member of the swarm. It will be useful for increasing the swarm size slightly when there are only 2 Kheperas.

### Centroid Estimator Launch File
This file launches two Rosserial python nodes to connect to 2 Kheperas and publish their positions and estimates onto the */khepera_pose* and */centroid_estimate* ROS topics, respectively.

### Usage
