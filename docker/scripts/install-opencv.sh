# ====================================
# Install OpenCV
# ====================================

CURRENT_DIR=$1

cd $CURRENT_DIR/build_tmp
export OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz
ARCH=$(uname -i)
echo "ARCH:  $ARCH"

set -e

# remove previous OpenCV installation if it exists
sudo apt-get purge -y '.*opencv.*' || echo "previous OpenCV installation not found"

mkdir -p opencv
cp ${OPENCV_DEB} opencv/${OPENCV_DEB}
cd opencv
tar -xzvf ${OPENCV_DEB}

# install the packages and their dependencies
sudo dpkg -i --force-depends *.deb
sudo apt-get update 
sudo apt-get install -y -f --no-install-recommends
sudo dpkg -i *.deb
sudo rm -rf /var/lib/apt/lists/*
sudo apt-get clean

# remove the original downloads
cd ../
rm -rf opencv

# manage some install paths
PYTHON3_VERSION=`python3 -c 'import sys; version=sys.version_info[:3]; print("{0}.{1}".format(*version))'`

if [ $ARCH = "aarch64" ]; then
	local_include_path="/usr/local/include/opencv4"
	local_python_path="/usr/local/lib/python${PYTHON3_VERSION}/dist-packages/cv2"

	if [ -d "$local_include_path" ]; then
		echo "$local_include_path already exists, replacing..."
		sudo rm -rf $local_include_path
	fi
	
	if [ -d "$local_python_path" ]; then
		echo "$local_python_path already exists, replacing..."
		sudo rm -rf $local_python_path
	fi
	
	sudo ln -s /usr/include/opencv4 $local_include_path
	sudo ln -s /usr/lib/python${PYTHON3_VERSION}/dist-packages/cv2 $local_python_path
	
elif [ $ARCH = "x86_64" ]; then
	opencv_conda_path="/opt/conda/lib/python${PYTHON3_VERSION}/site-packages/cv2"
	
	if [ -d "$opencv_conda_path" ]; then
		echo "$opencv_conda_path already exists, replacing..."
		sudo rm -rf $opencv_conda_path
		sudo ln -s /usr/lib/python${PYTHON3_VERSION}/site-packages/cv2 $opencv_conda_path
	fi
fi

# test importing cv2
echo "testing cv2 module under python..."
python3 -c "import cv2; print('OpenCV version:', str(cv2.__version__)); print(cv2.getBuildInformation())"

apt list | grep opencv