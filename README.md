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

