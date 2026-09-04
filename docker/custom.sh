# clone acados repo and build it
git clone --recurse-submodules https://github.com/acados/acados.git /opt/acados
cd /opt/acados
git checkout 0343432156d8dd82ff1bb30a0986e94c32c5cc3e
git submodule update --init --recursive
mkdir -p /opt/acados/build
cd /opt/acados/build
cmake -DCMAKE_BUILD_TYPE=Release .. # -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_QORE=ON -DACADOS_WITH_OOQP=ON -DACADOS_WITH_QPDUNES=ON -DACADOS_WITH_OSQP=ON -DACADOS_WITH_OPENMP=ON
make install -j8

# install acados python interface
pip install -e /opt/acados/interfaces/acados_template --ignore-installed

# install t_renderer
rm -f /opt/acados/bin/t_renderer
ACADOS_SOURCE_DIR=/opt/acados python -c "from acados_template import get_tera; get_tera(force_download=True)"

# write necessary environment variables to .bashrc
echo "export ACADOS_SOURCE_DIR=/opt/acados" >> /root/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/acados/lib" >> /root/.bashrc

# make acados libs discoverable without requiring LD_LIBRARY_PATH at runtime
echo "/opt/acados/lib" > /etc/ld.so.conf.d/acados.conf
ldconfig

# TODO: remove once [https://github.com/ros2/ros2_tracing/issues/211] is solved in released version
# overwrite released ros2_tracing packages with fork to support
# 'message-link instrumentation' and 'dual-session mode' in jazzy
cd /docker-ros/ws
git clone --branch jazzy-ika https://github.com/RaphvK/ros2_tracing.git src/ros2_tracing
rosdep update && rosdep install -y -i --from-paths src/ros2_tracing
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' --allow-overriding tracetools --allow-overriding tracetools_launch
rm -r src/ros2_tracing log build
