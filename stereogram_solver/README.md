# VTK

Install `vtk` for `viz`

```
sudo apt-get install libvtk6.3 libvtk6-dev
```

# PCL

Install `pcl`

```
sudo apt-get install libpcl-dev
```

# OpenCV

Download [opencv-3.4.16](https://github.com/opencv/opencv/releases/tag/3.4.16) source code

Extract into project root directory and rename it into opencv

In project root directory create opencv_build directory

Your project should look like
```
root
  |
  |-opencv
  |-opencv_build
```

## Building OpenCV

From opencv_build configure build files
```
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../opencv/
make -j4
make install
```

# Project

Create `build` directory

```
cd build
cmake ..
```

For debug build use
```
cmake .. -DDEBUG=ON
```

Inside `build`
```
make
```

To run execute `main` inside `build`

# Note

Terminal window *should* be active to read input

GUI does not handle input (use Ctrl-C or esc to exit)

Key input is not blocked during processing image but discarded even if programm ouputs `Waiting for input`

For directory path you *must* provide full name from root

e.g.
```
/home/username/images
```
or
```
/home/username/images
```
not
```
./images
```

# Files
## main.cpp
Entry point

## file_list.hpp/.cpp
Provide list of names for files in directory

## stereogram_solver.hpp/.cpp
Algorithms for image processing such as depth calculation and so on

## render.hpp/.cpp
Responsible for window updating

## log.hpp/log.cpp
logging function for debug