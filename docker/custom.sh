# clone acados repo and build it
git clone --recurse-submodules https://github.com/acados/acados.git /opt/acados
mkdir -p /opt/acados/build
cd /opt/acados/build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4

# install acados python interface
pip install -e /opt/acados/interfaces/acados_template

# install t_renderer
rm -f /opt/acados/bin/t_renderer
curl -L -o /opt/acados/bin/t_renderer https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux
chmod +x /opt/acados/bin/t_renderer

# write necessary environment variables to .bashrc
echo "export ACADOS_SOURCE_DIR=/opt/acados" >> /root/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/acados/lib" >> /root/.bashrc
