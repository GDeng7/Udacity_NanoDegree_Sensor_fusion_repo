# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Python >= 3.0

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Result
The test is run by script `test_loop.py` and results are stored under `./Result/Test_results.txt`.

In this test, I select KNN match selection (k=2) and performed descriptor distance ratio filtering with t=0.8.

The results can be shown as in following table:


|No. | Detector + Descriptor | Total Keypoints | Total Matches points | Total Time (ms) | 
|:---:|:-----:|:-----:|:-----:|:-----:|
|1 | SHITOMASI + BRISK |1179 |767 |2373.05 |
|2 | SHITOMASI + BRIEF |1179 |944 |182.194 |
|3 | SHITOMASI + ORB |1179 |907 |202.928 |
|4 | SHITOMASI + FREAK |1179 |768 |460.79 |
|5 | SHITOMASI + AKAZE |N/A |N/A |N/A |
|6 | SHITOMASI + SIFT |1179 |927 |327.785 |
|7 | HARRIS + BRISK |248 |142 |2437.63 |
|8 | HARRIS + BRIEF |248 |173 |543.394 |
|9 | HARRIS + ORB |248 |160 |549.27 |
|10 | HARRIS + FREAK |248 |144 |487.566 |
|11 | HARRIS + AKAZE |N/A |N/A |N/A |
|12 | HARRIS + SIFT |248 |163 |343.539 |
|13 | FAST + BRISK |1491 |899 |3159.49 |
|14 | FAST + BRIEF |1491 |1099 |67.8057 |
|15 | FAST + ORB |1491 |1081 |91.7741 |
|16 | FAST + FREAK |1491 |878 |378.239 |
|17 | FAST + AKAZE |N/A |N/A |N/A |
|18 | FAST + SIFT |1491 |1046 |226.954 |
|19 | BRISK + BRISK |2762 |1570 |5645.19 |
|20 | BRISK + BRIEF |2762 |1704 |3411.57 |
|21 | BRISK + ORB |2762 |1679 |2619.6 |
|22 | BRISK + FREAK |2762 |1524 |2942.23 |
|23 | BRISK + AKAZE |N/A |N/A |N/A |
|24 | BRISK + SIFT |2762 |1747 |4229.59 |
|25 | ORB + BRISK |1161 |751 |2404.55 |
|26 | ORB + BRIEF |1161 |545 |192.671 |
|27 | ORB + ORB |1161 |575 |216.047 |
|28 | ORB + FREAK |1161 |420 |507.68 |
|29 | ORB + AKAZE |N/A |N/A |N/A |
|30 | ORB + SIFT |1161 |897 |1573.23 |
|31 | AKAZE + BRISK |1670 |1215 |2798.41 |
|32 | AKAZE + BRIEF |1670 |1266 |560.503 |
|33 | AKAZE + ORB |1670 |1264 |583.82 |
|34 | AKAZE + FREAK |1670 |1187 |852.139 |
|35 | AKAZE + AKAZE |1670 |1202 |1003.34 |
|36 | AKAZE + SIFT |1670 |1334 |725.309 |
|37 | SIFT + BRISK |1386 |592 |2962.05 |
|38 | SIFT + BRIEF |1386 |702 |932.083 |
|39 | SIFT + ORB |1386 |677 |979.139 |
|40 | SIFT + FREAK |1386 |593 |1250.76 |
|41 | SIFT + AKAZE |N/A |N/A |N/A |
|42 | SIFT + SIFT |1386 |667 |1348.94 |

## Top three detector & descriptor pairs

As can be fround from the table above, the top 3 combinations of detector and descriptor are listed as below based on total time.

|No. | Detector + Descriptor | Total Keypoints | Total Matches points | Total Time (ms) | 
|:---:|:-----:|:-----:|:-----:|:-----:|
|1 | FAST + BRIEF |1491 |1099 |67.8057 |
|2 | FAST + ORB |1491 |1081 |91.7741 |
|3 | SHITOMASI + BRIEF |1179 |944 |182.194 |
