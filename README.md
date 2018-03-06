This is my research code for the paper ["A Learning Approach for Real-Time
Temporal Scene Flow Estimation from LIDAR
Data"](http://www.aushani.com/pdfs/aushani-2017a.pdf) presented at ICRA 2017.
Please contact me (aushani@gmail.com) with any questions or comments. Please
cite the paper if this code is useful in your work.
[(bibtex)](http://www.aushani.com/bibtex/aushani-2017a.txt)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=6p_JQYnAe24" target="_blank">
  <img src="http://img.youtube.com/vi/6p_JQYnAe24/0.jpg" alt="youtube video" width="480" height="360" border="10" />
</a>

## Getting Started

If you have any questions, please contact aushani@gmail.com. Please note that
the models here (background filter, occupancy constancy) are trained for the
KITTI dataset.

### Dependencies

Cuda is assumed to be already installed (you need a GPU to run the code). Other
dependencies are bazel, Qt4, and OpenSceneGraph. You also need the KITTI data.
The bootstrap.sh script takes care of these except for cuda.

### bootstrap.sh

To get started, run:

```shell
$ ./bootstrap.sh
```

You will be prompted for:

* Bazel

  Build system to compile the project

* Qt4

  from apt-get

* OpeneSceneGraph

  Used for rendering, locally built

* KITTI Data

  Prepackaged in the directory structure that the code is expecting. You also
  have the option of downloading a small snippet of it if bandwidth or diskspace
  is a concern.

### Compile the Project

To compile everything, go to the cc directory and build with bazel:

```shell
$ cd cc
$ bazel build //...
```

### Running

After building, you can run with a command like:

```shell
$ bazel-bin/app/flow/viewer
```

This assumes that the data was put in ~/data/tsf_data. Otherwise:

```shell
$ bazel-bin/app/flow/viewer --tsf-data-dir path/to/tsf_data
```
