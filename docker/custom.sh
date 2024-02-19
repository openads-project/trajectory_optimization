# create install directory and clone repository in temporary folder
cd /opt
mkdir acados
cd /tmp
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# build the library and include folders via cmake at the created install directory
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_INSTALL_DIR=/opt/acados ..
make install -j4

# remove files from temporary folder
cd /tmp
rm -rf acados