# Author: Andres Ricardo Garcia Escalante 
# Github: https://github.com/AndresGarciaEscalante

################ Emergency Brake Driver Assistant System on Jetson TX2 #######################
## This script provides dependecies needed to execute the EBDAS on the Jetson TX2 (Without a docker container). Please respect the 
## versions of these dependecies as they are selected based on: 
# https://www.tensorflow.org/install/source#tested_build_configurations
# https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform-release-notes/tf-jetson-rel.html
# https://github.com/tensorflow/addons

## This can be also be installed in a docker container, just remove the sudo in every linux command: 
## Based on: https://ngc.nvidia.com/catalog/containers/nvidia:l4t-tensorflow

# Pull Image
# sudo docker pull nvcr.io/nvidia/l4t-tensorflow:r32.5.0-tf2.3-py3

# Create Container 
# sudo nvidia-docker run --name TL_EBDA -v /home/nvidia/Desktop/cartec/TL_EBDA:/workspace -it -p 8888:8888 nvcr.io/nvidia/l4t-tensorflow:r32.5.0-tf2.3-py3

################ Tensorflow 2.3.1 Installation ########################
# Installation based on: https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html#benefits
# Errors fixed with: https://github.com/h5py/h5py/issues/1461
os.system('sudo apt-get update')
os.system('sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran')
os.system('sudo apt-get install python3-pip')
os.system('sudo pip3 install -U pip testresources setuptools==49.6.0')
os.system('sudo apt-get install pkg-config libhdf5-100 libhdf5-dev')
os.system('sudo pip3 install -U numpy==1.19.4 future==0.18.2 mock==3.0.5 h5py==2.10.0 keras_preprocessing==1.1.1 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11')
os.system('sudo pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v45 tensorflow==2.3.1+nv20.12')
################ End Tensorflow 2.3.1 Installation ####################

################# Bazel 3.1.0 Installation ############################
# Installation based on : https://qengineering.eu/install-tensorflow-2-lite-on-jetson-nano.html
os.system('sudo apt-get update')
os.system('sudo apt-get upgrade')
os.system('sudo apt-get install python-pip python3-pip -y')
os.system('sudo apt-get install build-essential zip unzip curl -y')
os.system('sudo apt-get install openjdk-11-jdk -y')
os.system('wget https://github.com/bazelbuild/bazel/releases/download/3.1.0/bazel-3.1.0-dist.zip')
os.system('unzip -d bazel bazel-3.1.0-dist.zip')
os.chdir('./bazel') 
os.system('sudo apt install nano')
os.system('nano scripts/bootstrap/compile.sh -c') # add in the line 154 with " -J-Xmx1600M" 
os.system('env EXTRA_BAZEL_ARGS="--host_javabase=@local_jdk//:jdk" bash ./compile.sh')
os.system('sudo cp output/bazel /usr/local/bin/bazel')
os.chdir('./..')
os.system('rm bazel-3.1.0-dist.zip')
os.system('rm -rf bazel')
os.system('bazel --version')
################## End Bazel 3.1.0 Installation ######################

#################### Tensorflow Addons 0.13.0 Installation ############################
os.system('wget https://github.com/tensorflow/addons/archive/refs/tags/v0.13.0.tar.gz')
os.system('tar -xvzf v0.13.0.tar.gz')
os.system('pip3 install addons-0.13.0/')
os.system('rm -rf addons-0.13.0/')
os.system('rm -rf v0.13.0.tar.gz')
#################### End Tensorflow Addons 0.13.0 Installation ###################

#################### Tensorflow Object Detection API 2.3.0 ############################
os.system('curl -OL https://github.com/protocolbuffers/protobuf/releases/download/v3.17.3/protoc-3.17.3-linux-aarch_64.zip')
os.system('sudo unzip protoc-3.17.3-linux-aarch_64.zip -d /usr/local bin/protoc')
os.system('sudo unzip protoc-3.17.3-linux-aarch_64.zip -d /usr/local include/*')
os.system('rm -f protoc-3.17.3-linux-aarch_64.zip')
os.system('python3 -m pip install -U pip')
os.system('wget https://github.com/tensorflow/models/archive/refs/tags/v2.3.0.tar.gz')
os.system('tar -xvzf v2.3.0.tar.gz')
os.system('rm -rf v2.3.0.tar.gz')
os.chdir('./models-2.3.0/research') 
os.system('sudo protoc object_detection/protos/*.proto --python_out=.')
os.system('python3 -m pip install --use-feature=2020-resolver .')
os.system('sudo apt install git-all -y')
os.system('pip install git+https://github.com/google-research/tf-slim')
os.system('python3 object_detection/builders/model_builder_tf2_test.py')
os.chdir('./../..')
#################### End Tensorflow Object Detection API 2.3.0 ############################

#################### Jupyter notebook Installation ########################################
os.system('pip install jupyter')
