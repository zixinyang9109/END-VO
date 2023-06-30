# END-VO

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

## Run
```
bin/run_end_stereo --log_dir=.
or
bin/run_end_stereo --logtostderr #save the poses to your current folder
```

## Mesh

## Evaluation

## Suggestions

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
