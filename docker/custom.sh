# clone acados repo and build it
git clone --recurse-submodules https://github.com/acados/acados.git /opt/acados
cd /opt/acados
git checkout faf2f79b73b58365d81b9e0c73e81b658076803b
mkdir -p /opt/acados/build
cd /opt/acados/build
cmake -DCMAKE_BUILD_TYPE=Release .. # -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_QORE=ON -DACADOS_WITH_OOQP=ON -DACADOS_WITH_QPDUNES=ON -DACADOS_WITH_OSQP=ON -DACADOS_WITH_OPENMP=ON
make install -j8

# install acados python interface
pip install -e /opt/acados/interfaces/acados_template

# install t_renderer
rm -f /opt/acados/bin/t_renderer
curl -L -o /opt/acados/bin/t_renderer https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux
chmod +x /opt/acados/bin/t_renderer

# write necessary environment variables to .bashrc
echo "export ACADOS_SOURCE_DIR=/opt/acados" >> /root/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/acados/lib" >> /root/.bashrc
