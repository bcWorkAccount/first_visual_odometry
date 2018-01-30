# FVO - First Visual Odometry
This is FVO SLAM, a stereo VO code. It is designed for stereo module.It uses the ORBExtractor as freont-end. It will add the backend in the future. 
The code is the first visual odometry and our team are all beginners, we can't guarantee its the best modularization and launch in high performance although we have referenced several open source frameworks and try our best to get to it.

# Dependency
If you are using ubuntu, all the dependencies are here:

- Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin 
- Eigen3: sudo apt-get install libeigen3-dev
- g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake 
- OpenCV: sudo apt-get install libopencv-dev
- glog (for logging): sudo apt-get install libgoogle-glog-dev
- PCL: point cloud library
- sophus: Lie group and Li algebra calculating

# Compile
You can compile all the things as following instruction.
```
mkdir cmake-build-debug
cd cmake-build-debug
cmake ..
make 
```
and then the executable file will be compiled and output to the bin directory and it has name 'fvo-stereo'

# Examples
You can put stereo data into fvo-stere. We support the KITTI dataset now. Launch the program by typing:
```
bin/fvo-stereo  ./config/kitti.yaml  ./Config/groundtruth.txt  <sequence_data_dir>
```
to run the fvo stereo VO case.
we have already provided the kitti.yaml at config directory and you should download the sequence of kitti data set and type it as the <sequence_data_dir> parameter, like: /home/hadoop/kitti_dataset/sequence/08 . 
We skip the groundtruth.txt parameters at this version of project because KITTI doesn't provide these data so you can type any thing for it.


# Other things
We reference following open soruce framework when we deisign the system architecture of fvo project because it's the first visual odometry and our team are Newbie:
- ORB-SLAM2 
- slambook
- ygz-stereo-inertial
- openslam

Contact me (shihezichen@live.cn) or Qianli (xxx) if you have any question.


