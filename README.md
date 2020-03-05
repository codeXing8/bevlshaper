### bevlshaper
Algorithm for bird's-eye-view L-shape fitting in 3D LIDAR point clouds from traffic scenarios

![conda](https://img.shields.io/badge/Conda-4.7.5-green.svg)
![python](https://img.shields.io/badge/Python-3.8.1-yellow.svg)
![numpy](https://img.shields.io/badge/NumPy-1.18.1-blue.svg)

---

:hammer: **UNDER DEVELOPMENT** :wrench:

#### Foreword
This project is inspired by the paper [Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners](https://www.ri.cmu.edu/wp-content/uploads/2017/07/Xiao-2017-Efficient-L-Shape-Fitting.pdf).
As in the paper, a search tree algorithm is used for segmentation after point cloud filtering.
This step is inspired by [Moving object classification using horizontal laser scan data](https://www.researchgate.net/profile/Huijing_Zhao/publication/224557150_Moving_object_classification_using_horizontal_laser_scan_data/links/00b7d520b05aa1a131000000/Moving-object-classification-using-horizontal-laser-scan-data.pdf).
Within found clusters, L-shapes are detected. Therefore, rectangles are searched and reduced to L-shapes (oriented towards the sensing vehicle) afterwards.
For easy prototyping and modelling, Python and the NumPy library are used instead of a more computationally powerful language.
<p align="center">
    <img src="pcl_lshapes.gif" alt="Clustered point cloud data from bird's-eye-view with fitted L-shapes/rectangles"/>
</p>

#### Prepare environment
```bash
conda create --name kitti -y python=3 \
&& conda activate kitti \
&& git clone https://github.com/scud3r1a/exploreKITTI.git \
&& pip install numpy pykitti matplotlib opencv-python opencv-contrib-python moviepy \
&& mkdir frames
```

#### Run _bevlshaper_ on KITTI dataset scenes
```bash
time python bevlshaper.py 2011_09_26 0001
```