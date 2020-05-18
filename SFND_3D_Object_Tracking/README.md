# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Performance evaluation of Lidar TTC
The task is complete once several examples (2-3) have been identified and described in detail. The assertion that the TTC is off should be based on manually estimating the distance to the rear of the preceding vehicle from a top view perspective of the Lidar points.

Based on the results, I found one example of inaccurate Lidar based TTC estimation. The TTC estimation suddently goes to negative value and bounce back in consecutive frames.

|Frame No. |TTC |Xmin|
|:---:|:-----:|:----:|
|28 |11.8211 s| 5.95 m |
|29 |-23.5506 s |5.74 m |
|30 |8.14053 s|5.79 m |

This is due to the flucuration in the denominator that drives the whole value to negative. Because the vehicle is moving slowly, the average value of minimum range in current frame and previous frame are close and its difference could possibly be nagetive. 

## Performance evaluation of different detector/descriptor

Based on results from previous project, the 3 best combinations are FAST/BRIEF, FAST/ORB and SHITOMASI/BRIEF. The comparision of camera based TTC with these three combinations are shown in below.

|Frame No. |Lidar TTC |FAST/BRIEF | FAST/ORB | SHITOMASI/BRIEF|
|:----:|:-----:|:----:|:-----:|:-----:|
|1 |12.3245 |10.8026 |11.0544 |14.6756|
|2 |13.3658 |11.0063 |10.6351 |14.4019 |
|3 |16.3633 |14.1559 |13.4008 |9.73978 |
|4 |14.038 |14.3886 |12.7761 |14.982 |
|5 |12.6624 |19.9511 |18.0923 |12.7503 |
|6 |13.6958 |13.293 |12.9892 |13.2703 |
|7 |13.6398 |12.2182 |12.4642 |15.2664 |
|8 |13.8336 |12.7596 |11.3522 |12.0847 |
|9 |12.0968 |12.6 |12.1119 |11.8703 |
|10 |11.8623 |13.4637 |13.4169 |12.6285 |
|11 |11.9584 |13.6717 |13.4175 |11.8507 |
|12 |9.83108 |10.9087 |10.4239 |11.7642 |
|13 |9.37912 |12.3705 |12.0462 |11.7197 |
|14 |9.43773 |11.2431 |11.0103 |11.3557 |
|15 |8.25041 |11.8747 |11.3841 |12.1983 |
|16 |8.84877 |11.8398 |11.6 |8.23961 |
|17 |11.2891 |7.92013 |7.56799 |11.1382 |
|18 |8.57706 |11.554 |10.3912 |8.43097 |

Compared to the Lidar based TTC, the camera-based TTC is not stable. There are few values of camera-based TTC that change back and forth with more than 2 seconds in consecutive frames, which is not as consistent as TTC estimation using Lidar. This is because the calculation of TTC using camera used the median distance ratio and it assumed that all associated points are in the same plane. However, there are cased when some wrong matched points are counted as mactched keypoints and lead to inconsistency of TTC estimation. The comparison plot can be shown as below.

<img src="images/ttc.png" width="500" height="200" />


