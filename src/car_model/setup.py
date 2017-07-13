## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/car_publisher.py', 'src/video_converter.py',
        'src/stream_converter.py'],
    packages=['car_model'],
    package_dir={'': 'src'},
)

setup(**setup_args)
