# FVO - First Visual Odometry
This is FVO SLAM, a stereo VO code. It is designed for stereo module like vi-sensor.It uses the ORBExtractor and LK optical flow as freont-end. It will add the backend in the future. Feel free to try it in datasets and your own sensors.

The code is the first visual odometry and our team are all beginners, we can't guarantee its the best modularization and launch performance although we reference many open source frameworks and try our best to do it.

# Dependency
If you are using ubuntu, all the dependencies are here:

- Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin 
- Eigen3: sudo apt-get install libeigen3-dev
- g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake 
- OpenCV: sudo apt-get install libopencv-dev
- glog (for logging): sudo apt-get install libgoogle-glog-dev

# Compile
You can compile all the things as following instruction.
```
cd build-makes-debug
cmake ..
make 
```

# Examples
You can put stereo data into fvo-stere. We support the KITTI dataset now. Launch the program by typing:
```
bing/fvo-stereo  ./Config/kitti.yaml  ./Config/groundtruth.txt  <sequence_date_dir>
```
to run the fvo stereo VO case.
We skip the groundtruth.txt parameters at this version of project because KITTI doesn't provide these data so you can input any thing for it.




# Other things
We reference following open soruce framework when we deisign the system architecture of fvo project because it's the first visual odometry and our team are Newbie:
- ORB-SLAM2 
- slambook
- ygz-stereo-inertial

Contact me (shihezichen@live.cn) or Qianli (xxx) if you have any question.


