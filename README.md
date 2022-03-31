# LIFT-SLAM

[WORKDOC](https://docs.google.com/document/d/1vHbQIUsuTtM1g_gCduerTWzS3t2Z4YODiUfZuolXTvg/edit
)

## Installation

Things to apt install

```
sudo apt install libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libeigen3-dev
```

### ORB-SLAM2

- #### OpenCV

- #### Pangolin

  ```
  cd third-party/Pangolin-0.6
  mkdir build
  cd build
  cmake ..
  cmake --build .
  sudo make install
  ```

- #### Eigen
