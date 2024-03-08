import os
from glob import glob
from setuptools import setup

package_name = 'acados_tp_ocp_def'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[(os.path.join('share', package_name), ['package.xml']),
                (os.path.join('share', package_name,
                              'launch'), glob('launch/*launch.[pxy][yma]*'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guido Küppers',
    maintainer_email='guido.kueppers@ika.rwth-aachen.de',
    description='This package stores the ACADOS Optimal-Control-Problem (OCP) definition for an NMPC-based trajectory-planner for automated vehicles',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ['generate_acados_ocp = acados_tp_ocp_def.generate_ocp:main'],
    },
)
