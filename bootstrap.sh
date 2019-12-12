echo Setting up environment for TSF
echo

# cuda
echo Please install cuda on your own from the NVIDIA website
echo

# Install bazel #######################
read -r -p "Install Bazel 0.4.5 [y/N] " response
case "$response" in
  [yY][eY][eS]|[yY])
    echo Installing bazel

    wget https://github.com/bazelbuild/bazel/releases/download/0.4.5/bazel-0.4.5-installer-linux-x86_64.sh

    chmod +x bazel-0.4.5-installer-linux-x86_64.sh
    sudo ./bazel-0.4.5-installer-linux-x86_64.sh

    ;;
  *)
    echo Not Installing Bazel
esac
echo
#######################################


# Install qt4 ########################
read -r -p "Install Qt4 [y/N] " response
case "$response" in
  [yY][eY][eS]|[yY])
    echo Installing Qt4
    sudo apt-get install libqt4-dev
    ;;
  *)
    echo Not Installing Qt4
esac
echo
#######################################

# OpenSceneGraph
read -r -p "Build OpenSceneGraph [y/N] " response
case "$response" in
  [yY][eY][eS]|[yY])
    echo Building OpenSceneGraph

    git submodule update --init --recursive

    cd OpenSceneGraph
    mkdir build -p
    cd build
    cmake -D CMAKE_C_FLAGS="-fPIC" \
          -D CMAKE_CXX_FLAGS="-fPIC" \
          -D CMAKE_INSTALL_PREFIX="local_install" \
          -D BUILD_OSG_APPLICATIONS="OFF" \
          -D FFMPEG_LIBAVCODEC_INCLUDE_DIRS="" \
          ../

    make -j $(nproc)
    make install
    cd ../..
    ;;
  *)
    echo Not building OpenSceneGraph
esac
echo
#######################################

# Get data
read -r -p "Get Data [y/N] " response
case "$response" in
  [yY][eY][eS]|[yY])
    read -p "Enter a path for storing the data: " -i "$HOME" -e path

    mkdir -p $path
    cd $path

    echo Downloading data

    if wget http://robots.engin.umich.edu/~aushani/tsf_data.tar.gz
    then
      echo Got tarball
    else
      echo Couldn\'t connect to http://robots.engin.umich.edu/~aushani/tsf_data.tar.gz
      echo Trying secondary address
      if wget http://www.aushani.com/files/tsf_data.tar.gz
      then
        echo Got tarball
      else
        echo Couldn\'t connect to http://www.aushani.com/files/tsf_data.tar.gz
        echo Please contact aushani@gmail.com for assistance
        break
      fi
    fi

    tar xvpf tsf_data.tar.gz

    ;;
  *)
    # Get data snippet
    read -r -p "Get small snippet of data [y/N] " response
    case "$response" in
      [yY][eY][eS]|[yY])
        read -p "Enter a path for storing the data: " -i "$HOME" -e path

        mkdir -p $path
        cd $path

        echo Downloading data

        if wget http://robots.engin.umich.edu/~aushani/tsf_data_small.tar.gz
        then
          echo Got tarball
        else
          echo Couldn\'t connect to http://robots.engin.umich.edu/~aushani/tsf_data_small.tar.gz
          echo Trying secondary address
          if wget http://www.aushani.com/files/tsf_data_small.tar.gz
          then
            echo Got tarball
          else
            echo Couldn\'t connect to http://www.aushani.com/files/tsf_data_small.tar.gz
            echo Please contact aushani@gmail.com for assistance
            break
          fi
        fi

        tar xvpf tsf_data_small.tar.gz

        ;;
      *)
        echo Not getting data
    esac
    echo
    #######################################
esac
echo
#######################################

