# Visual-Odometry
### Implementation of basic visual odometry pipeline on the [Sahayak](https://www.aidbots.in/#/ ) robot in ROS in a custom Gazebo environment
## 1.  2D to 2D Motion Estimation:
* Detection of keypoints and calculation of descriptors is done using the [SIFT](https://docs.opencv.org/master/da/df5/tutorial_py_sift_intro.html) algorithm.
* Feature matching is done using [Brut-Force Matcher](https://docs.opencv.org/master/da/df5/tutorial_py_sift_intro.html).

  <img src="https://user-images.githubusercontent.com/64685403/124266666-e32c5000-db54-11eb-8b63-aee21882d407.png" width="400">

  <img src="https://user-images.githubusercontent.com/64685403/124284248-6c9a4d00-db6a-11eb-909b-597a79283cae.png" width="400">
  
## 2.  2D to 2D Motion Estimation using optical flow through Lucas Kanade Trackers:

## 3.  3D to 2D Motion Estimation:
* Detection of keypoints and calculation of descriptors is done using the [ORB](https://docs.opencv.org/master/d1/d89/tutorial_py_orb.html) algorithm.
* Feature matching is done using [Flann Based Matcher](https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html).
* 3D Point cloud and images from camera is used and the transformation is calculated using Perspective-n-Point Method
  <img src="https://user-images.githubusercontent.com/64685403/124307661-c4927d00-db85-11eb-954a-b16af61d68fe.png" width="400">




 

