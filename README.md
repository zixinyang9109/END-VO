# END-VO

## Introduction

Code of Endoscope Localization and Dense Surgical Scene Reconstruction for Stereo Endoscopy by Unsupervised Optical Flow and Kanade-Lucas-Tomasi Tracking. This project allows me to master the pipeline of SLAM.


https://github.com/zixinyang9109/END-VO/assets/126973110/ed34978f-c5d3-4050-b67c-cfa3ad367ee6



## Setup
### Install dependences required in the CMakeLists.txt
OpenCV 3.1 https://github.com/opencv/opencv/archive/3.1.0.zip
```
unzip opencv-3.1.0.zip
cd opencv-3.1.0
mkdir build
cd build 
cmake ..
make
sudo make install
```
pangolin
```
sudo apt-get install libglew-dev  
sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
sudo apt-get install git
git clone https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin
mkdir build
cd build 
cmake ..
make
sudo make install
```

Sophus
```
git clone http://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build 
cmake ..
make
make install
```

G20
```
sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build 
cmake ..
make
sudo make install
```

glog
```
sudo apt update
sudo apt install libgoogle-glog-dev
```

gtest
```
git clone https://github.com/google/googletest.git
cd googletest
mkdir build
cd build
cmake ..
make
sudo make install

```

csparse
```
sudo apt-get install libsuitesparse-dev
```

### Complie the project
```
mkdir build
cd build/
cmake ..
make -j4
sudo make install
```


## Dataset
We used the SCARED dataset (https://arxiv.org/abs/2101.01133).
A sample of the processed [dataset](https://drive.google.com/file/d/1gj_dGt9zgMFTODcUD4dDv-MnWeSFqcC_/view?usp=sharing).

## Run
Please change the dataset_dir and depth_dir in the config/endoscope0103.yaml and run:

```
bin/run_end_stereo --log_dir=. #save the poses to a specific folder
or
bin/run_end_stereo --logtostderr #save the poses to your current folder
```

## Mesh
Please refer to http://www.open3d.org/docs/0.12.0/tutorial/pipelines/rgbd_integration.html and Endo-Depth-and-Motion](https://github.com/UZ-SLAMLab/Endo-Depth-and-Motion/tree/main)

## Evaluation

For pose evaluation, please install evo (https://github.com/MichaelGrupp/evo) and go to the eva folder.
```
python evo_mydata.py
python plot_all.py
```
Fore reconstruction evaluation, please use (https://www.danielgm.net/cc/).


## Citation

```bibtex
@inproceedings{yang2022endoscope,
  title={Endoscope Localization and Dense Surgical Scene Reconstruction for Stereo Endoscopy by Unsupervised Optical Flow and Kanade-Lucas-Tomasi Tracking},
  author={Yang, Zixin and Lin, Shan and Simon, Richard and Linte, Cristian A},
  booktitle={2022 44th Annual International Conference of the IEEE Engineering in Medicine \& Biology Society (EMBC)},
  pages={4839--4842},
  year={2022},
  organization={IEEE}
}
```


## Acknowledgements
- [Slambook2](https://github.com/gaoxiang12/slambook2/tree/master) The project is built on the CH13 code.
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [Endo-Depth-and-Motion](https://github.com/UZ-SLAMLab/Endo-Depth-and-Motion/tree/main)
- [evo](https://github.com/MichaelGrupp/evo)

