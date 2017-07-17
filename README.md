CAUTION
====

The following instrucitons might not be up to date or complete.


CUDA-based implementations of Softassign and EM-ICP
====

CUDA-based implementations of Softassign and EM-ICP, CVPR2010 Demo
http://home.hiroshima-u.ac.jp/tamaki/study/cuda_softassign_emicp/

Toru Tamaki, Miho Abe, Bisser Raytchev, Kazufumi Kaneda, Marcos Slomp (Hiroshima University, Japan)
Contact address: tamaki@hiroshima-u.ac.jp

Wed Aug 27 19:11:45 JST 2014



Requirements
---

- CUDA 8.0
- PCL 1.8
- FLANN
- Boost
- lapack, blas


Usage
---

```
emicp [options]
```

This demo application finds R and t such that `min ||X - (R*Y+t) ||` for given 3D point sets X and Y.

At the start, the demo application popups a window where two point sets are shown.

### Options

#### Files of 3D point sets:
```
-pointFileX=filename
-pointFileY=filename
[string] Filename of two 3D point sets. Required.
```
Format of file: in each line, x y z coordinates are stored. That’s all. Number of points are specified with Xsize and Ysize options.

#### Reduction number of points:
```
-leaf_size=**
    [float] Leaf size to use for downsampling the pointclouds using PCL's voxel grid filter. default: 0.005
```


#### EM-ICP parameters:
````
-sigma_p2=***
    [float] initial value for the main loop. sigma_p2 <- sigma_p2 * sigma_factor  at the end of each iteration while sigma_p2 > sigam_inf. default: 0.01
-sigma_inf=***
    [float] minimum value of sigma_p2. default: 0.00001
-sigma_factor=***
    [float] facfor for reducing sigma_p2. default: 0.9
-d_02=***
    [float] values for outlier (see EM-ICP paper). default: 0.01
````

#### MISC:
````
-notimer
    No timer is shown.
````

####  Save and load R, t:
```
-saveRTtoFile=filename
    save the estimated rotation matrix and translation vector. default: nothing saved.
-loadRTfromFile=filename
    load initial values for rotation matrix R and translation vector t. default: no values are loaded, and set R=3x3 identity matrix, t=zero vector.
```

Save and load format: R and t are stored in row-wise.
```
r11 r12 r13
r21 r22 r23
r31 r32 r33
tx ty tz
```

Build the demo application
---

### prepare packages:
```
sudo apt-get install libpcl-1.7-all-dev libflann-dev libboost-all-dev ccache libatlas-dev libblas-dev liblapack-dev cmake
```

### build
```
mkdir build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_VERBOSE_MAKEFILE=1 ..
make
```




How to use the API
---

### interfaces:
```
void      emicp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targetX,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sourceY,float* h_R, float* h_t, registrationParameters param)
```

To use these functions,
compile your program with `emicp.cu`, along with  `3dregistration.h`, `cloud2data.cpp`, and `findRTfromS.cpp`.

### description
```
cloud_targetX, cloud_sourceY [Input]
```
Specify the pointers to pcl::PointCloud storing point sets X and Y.

```
float* h_R, float* h_t [Input/Output]
```
- On entry, initial values for rotation matrix R and translation vector t must be given. Usually, R=3x3 identity matrix, t=zero vector.
- On exit, estimated R and t are stored.
- Order of elements:
```
h_R[0], ..., h_R[8]: r11 r12 r13 r21 r22 r23 r31 r32 r33
h_t[0], ..., h_t[2] : tx,ty,tz
```

```
registrationParameters param [Input]
```
parameters for alignment. set values by yourself.
see 3dregistration.h and usage of demo (described above).


License: MIT
===
```
/*
  Copyright (c) 2010 Toru Tamaki

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
*/
```

We kindly ask for users to refer

- Toru Tamaki, Miho Abe, Bisser Raytchev, Kazufumi Kaneda, Marcos Slomp, "CUDA-based implementations of Softassign and EM-ICP," CVPR2010 Demo, 2009.
- Toru Tamaki, Miho Abe, Bisser Raytchev, Kazufumi Kaneda: "Softassign and EM-ICP on GPU", Proc. of The 2nd Workshop on Ultra Performance and Dependable Acceleration Systems (UPDAS), CD-ROM, 5 pages, 2010. http://dx.doi.org/10.1109/IC-NC.2010.60

in your paper publised by using our implementation. Thank you!


Acknowledgements
===
Dataset used in this demonstration is taken from the following website:
- The Stanford Bunny, Stanford University Computer Graphics Laboratory, The Stanford 3D Scanning Repository.
  http://graphics.stanford.edu/data/3Dscanrep/

