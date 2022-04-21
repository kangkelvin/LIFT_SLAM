# LIFT-SLAM

[WORKDOC](https://docs.google.com/document/d/1vHbQIUsuTtM1g_gCduerTWzS3t2Z4YODiUfZuolXTvg/edit
)

## Installation

Things to apt install

```
sudo apt install libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libeigen3-dev
```

### ORB-SLAM2

- #### OpenCV 3 

- #### Pangolin

  ```
  cd third-party/Pangolin-0.6
  mkdir build
  cd build
  cmake ..
  cmake --build .
  sudo make install
  ```

- #### Build

  ```
  ./build.sh
  ```

- #### Test

  ```
  ./third_party/ORB_SLAM2_OG/Examples/Monocular/mono_kitti third_party/ORB_SLAM2_OG/Vocabulary/ORBvoc.txt third_party/ORB_SLAM2_OG/Examples/Monocular/KITTI04-12.yaml data/04-Straight-Line-Drive
  
  ./third_party/ORB_SLAM2_OG/Examples/Monocular/mono_kitti third_party/ORB_SLAM2_OG/Vocabulary/ORBvoc.txt third_party/ORB_SLAM2_OG/Examples/Monocular/KITTI04-12.yaml data/06-2U-turns-same-road
  
  ./third_party/ORB_SLAM2_OG/Examples/Monocular/mono_kitti third_party/ORB_SLAM2_OG/Vocabulary/ORBvoc.txt third_party/ORB_SLAM2_OG/Examples/Monocular/KITTI17.yaml data/17-Curved-highway-drive
  
  ./third_party/ORB_SLAM2_OG/Examples/Monocular/mono_kitti_lift third_party/ORB_SLAM2_OG/Vocabulary/ORBvoc.txt third_party/ORB_SLAM2_OG/Examples/Monocular/KITTI04-12.yaml data/test
  ```
  
  

### LIFT

- #### OpenCV 3

- Several Python Packages

  ```
  pip install -r requirements.txt
  ```

  

- cudatoolkit (choose one)

  ```
  conda install -c anaconda cudatoolkit=10.2
  conda install -c anaconda cudatoolkit=11.0
  ```

  

- Build

  ```
  ```

  

- Test

  ```
  ```

  

## Digging Through ORB-SLAM2

**Path 0: Create SLAM system**

```
In ORB_SLAM2_OG/Examples/Monocular/mono_kitti.cc:53
ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

In ORB_SLAM2_OG/include/System.h:66
System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

Split to Path 1 and 2
```

**Path 1: Create tracker class `Tracking` and initialize feature extractor `ORBextractor` and feature matcher `ORBmatcher`**

```
In ORB_SLAM2_OG/include/System.h:146
Tracking* mpTracker;

In ORB_SLAM2_OG/src/Tracking.cc:46
Tracking::Tracking()

In ORB_SLAM2_OG/src/Tracking.cc:119
mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

In ORB_SLAM2_OG/include/ORBextractor.h:51
ORBextractor()
```

**Path 2: Grab new image frame**

```
In ORB_SLAM2_OG/include/System.h:82
In ORB_SLAM2_OG/src/System.cc:218
cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

In ORB_SLAM2_OG/src/System.cc:260
cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

In ORB_SLAM2_OG/include/Tracking.h:63
In ORB_SLAM2_OG/src/Tracking.cc:238
cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

Split to Path 3 and 4
```

**Path 3: Calculate ORB features**

```
In ORB_SLAM2_OG/src/Tracking.cc:260
mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

In ORB_SLAM2_OG/include/Frame.h:58 --> TO CHANGE
Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

In ORB_SLAM2_OG/src/Frame.cc:191
ExtractORB(0,imGray);

In ORB_SLAM2_OG/src/Frame.cc:247
(*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
```

**Path 4: Track frames**

```
In ORB_SLAM2_OG/src/Tracking.cc:262
Track();

In ORB_SLAM2_OG/src/Tracking.cc:267
void Tracking::Track()

In ORB_SLAM2_OG/src/Tracking.cc:309
bOK = TrackReferenceKeyFrame();

In ORB_SLAM2_OG/src/Tracking.cc:757
bool Tracking::TrackReferenceKeyFrame()

In ORB_SLAM2_OG/src/Tracking.cc:764 --> TO CHANGE
ORBmatcher matcher(0.7,true);

In ORB_SLAM2_OG/src/Tracking.cc:767 --> TO CHANGE
int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

In ORB_SLAM2_OG/src/Tracking.cc:775
Optimizer::PoseOptimization(&mCurrentFrame);
```



## Kelvin's Notes

After LIFT detector step, you get keypoints list, which is list[array], list length is based on `num_keypoint` and array length is (13, ) 

```
XYZS
X (col),      Y (row),       scale,     angle
[290.65896909 227.5774079    1.43058658 145.80065918]
kp_list
[2.90658969e+02 2.27577408e+02 1.43058658e+00 0.00000000e+00
 1.45800659e+02 8.38860800e+06 4.88620528e-01 0.00000000e+00
 4.88620528e-01 1.43058658e+00 0.00000000e+00 0.00000000e+00
 1.43058658e+00]
new_kp_list
[ 2.90658969e+02  2.27577408e+02  1.43058658e+00  2.63492310e+02
  1.45800659e+02  8.38860800e+06  4.88620528e-01  0.00000000e+00
  4.88620528e-01 -1.62137780e-01 -1.42136881e+00  1.42136881e+00
 -1.62137780e-01]
 
 # Keypoint List Structure Index Info
IDX_X, IDX_Y, IDX_SIZE, IDX_ANGLE, IDX_RESPONSE, IDX_OCTAVE = (
    0, 1, 2, 3, 4, 5)  # , IDX_CLASSID not used
IDX_a, IDX_b, IDX_c = (6, 7, 8)
# NOTE the row-major colon-major adaptation here
IDX_A0, IDX_A2, IDX_A1, IDX_A3 = (9, 10, 11, 12)
# # IDX_CLASSID for KAZE
# IDX_CLASSID = 13
```



ORB Extractor OG

```
Frame.h:134
// Vector of keypoints (original for visualization) and undistorted (actually used by the system).
// In the stereo case, mvKeysUn is redundant as images must be rectified.
// In the RGB-D case, RGB images can be distorted.
std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
std::vector<cv::KeyPoint> mvKeysUn;

Frame.h:149
// ORB descriptor, each row associated to a keypoint.
cv::Mat mDescriptors, mDescriptorsRight;
```

Sample LIFT Descriptor:

```
Shape: N_keypoints? x 128 (somehow the rows are slightly smaller than the keypoints detected, e.g. 500x 448)
Type: float
Min: ~0.44829488 (0?)
Max: ~3.996638	(4?)

[3.9814296 3.8330724 3.9752538 1.870607  3.6741552 3.220076  3.4282994
 3.9897435 3.982149  3.9826107 3.9860005 2.9128287 2.9598296 3.9839094
 2.3088315 3.9817607 3.982252  2.7871473 2.0272841 3.8354447 3.9806154
 3.9705548 2.5975702 3.9784756 3.978603  3.9763541 2.085818  2.9541464
 1.7380981 3.9888346 3.9767237 3.9831374 3.9825487 2.628675  3.9870822
 2.5765436 2.5518684 3.986904  3.9856734 3.9798574 3.9817815 3.9835348
 3.978641  3.460157  3.9861324 2.080081  3.9852614 3.9841223 3.980392
 3.9864476 3.9892669 3.8163924 2.8975306 3.9890027 3.9829898 2.896752
 3.745711  2.1278265 2.6270745 3.0749364 3.9789224 3.986961  3.980267
 3.9858186 2.4951105 3.0533814 2.8172586 2.8864088 3.980331  3.514426
 3.9805522 3.9804332 3.9724305 3.9818187 3.9844809 3.9747455 2.7868502
 3.3043754 3.9840539 2.5032835 3.9728477 3.986741  2.5758557 3.9782565
 2.3736613 3.9867187 3.9836123 3.9868941 3.988118  3.9768    3.982832
 3.9892614 2.0622864 3.5211103 2.4395885 3.586178  3.9868484 3.7243943
 3.4597392 2.7921312 3.9798508 3.7790918 3.987613  1.8380578 3.977255
 3.975647  3.9830012 3.9803987 3.6735442 3.7770061 3.9772475 3.9798052
 3.177387  3.3872764 3.9835331 3.9856157 2.3599544 3.9812367 3.9834867
 2.113241  3.988638  3.98475   2.3379493 3.9758124 3.2461414 3.9867764
 2.2136207 3.9859927]
```

Sample ORB Keypoints n Descriptor:

```
Keypoints: angle: 61.7395 classid: -1 octave: 0 x: 1185 y: 72 response: 109 size: 31

Shape: N_keypoints x 32, typical N_keypoints: 4000. Size of keypoints and descriptor matches unlike LIFT.
Type: CV_8U

[98, 193, 147, 158,  73, 128, 164, 101, 170,  17,  97,  72, 100, 242,  44, 214, 248, 239,  34,  65,  66, 218, 138, 105,  52,  50,  94,  42,   8,  26, 163, 142]
```

**idea** maybe just scale the lift descriptor to CV_8U mat to match ORB descriptor

