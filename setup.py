import distutils.sysconfig
import os
import os.path as osp
import re
import shutil
from setuptools import setup
import subprocess
import sys
import tarfile
import urllib.request

from pybind11.setup_helpers import Pybind11Extension, build_ext


# eigen
EIGEN_WORLD = 3
EIGEN_MAJOR = 3
EIGEN_MINOR = 7
EIGEN_VERSION = f'{EIGEN_WORLD}.{EIGEN_MAJOR}.{EIGEN_MINOR}'
EIGEN_URL = f'https://gitlab.com/libeigen/eigen/-/archive/{EIGEN_VERSION}/eigen-{EIGEN_VERSION}.tar.gz'


def parse_version_define(s, name):
    try:
        define_str = re.findall(fr'#define {name}[^\n]*', s)[0].split(' ')[2]
        return int(define_str)
    except Exception:
        return None


def check_eigen(include_dir):
    # check if eigen exists in include directory
    if not osp.isfile(osp.join(include_dir, 'Eigen', 'Core')):
        return False

    # check version
    version_file = osp.join(include_dir, 'Eigen', 'src', 'Core', 'util', 'Macros.h')
    if osp.isfile(version_file):
        with open(version_file) as f:
            eigen_version_header = f.read()
            eigen_world = parse_version_define(eigen_version_header, 'EIGEN_WORLD_VERSION')
            eigen_major = parse_version_define(eigen_version_header, 'EIGEN_MAJOR_VERSION')
            return eigen_world == EIGEN_WORLD and eigen_major >= EIGEN_MAJOR

    return False


def find_eigen(download=True):
    # collect search directories for eigen
    search_dirs = []

    # environment variable
    if 'EIGEN_DIR' in os.environ:
        search_dirs.append(os.environ['EIGEN_DIR'])

    # cmake
    try:
        cmake_result = subprocess.run(['cmake', '-P', osp.join('cmake', 'find_eigen.cmake')],
                                      capture_output=True, text=True)
        if cmake_result.returncode == 0:
            search_dirs.append(cmake_result.stderr.splitlines()[0])
    except Exception:
        pass

    # system include directories
    search_dirs.append(distutils.sysconfig.get_config_var('INCLUDEDIR'))
    if 'CPATH' in os.environ:
        search_dirs.append(os.environ['CPATH'])

    # search
    for base_dir in search_dirs:
        if not osp.isdir(base_dir):
            continue
        for sub_dir in ['', 'eigen3']:
            search_dir = osp.join(base_dir, sub_dir)
            if check_eigen(search_dir):
                return search_dir

    # download eigen if it was not found
    if download:
        download_dir = osp.abspath(osp.join('.download', 'eigen'))
        print(f"Downloading Eigen from '{EIGEN_URL}' to '{download_dir}'")

        # prepare directory
        try:
            if osp.isdir(download_dir):
                print("Clearing previous download")
                shutil.rmtree(download_dir)
            os.makedirs(download_dir, exist_ok=True)
        except Exception as e:
            print(f"Error: could not prepare download destination\n{e}")
            sys.exit(1)

        # download
        try:
            download_file = osp.join(download_dir, 'eigen.tar.gz')
            urllib.request.urlretrieve(EIGEN_URL, download_file)
        except Exception as e:
            print(f"Error: could not download Eigen\n{e}")
            sys.exit(1)

        # unpack
        print("Unpacking Eigen")
        try:
            tar = tarfile.open(download_file, "r:gz")
            tar.extractall(download_dir)
            tar.close()
        except Exception as e:
            print(f"Error: could not unpack Eigen\n{e}")
            sys.exit(1)

        # check
        eigen_dir = osp.join(download_dir, f'eigen-{EIGEN_VERSION}')
        if check_eigen(eigen_dir):
            return eigen_dir
        else:
            print("Downloaded Eigen has errors")
            sys.exit(1)

    # eigen not found
    print("Could not find Eigen installation")
    sys.exit(1)


EIGEN_INCLUDE_DIR = find_eigen()
print(f"Using Eigen from '{EIGEN_INCLUDE_DIR}'")


# define module
ext_modules = [
    Pybind11Extension(
        'motion3d',
        sources=[osp.join('src', 'python', 'motion3d_python_api.cpp')],
        include_dirs=[
            'include',
            EIGEN_INCLUDE_DIR
        ]
    )
]


# parse version
with open(osp.join('include', 'motion3d', 'motion3d.hpp')) as f:
    motion3d_version_header = f.read()
    MOTION3D_MAJOR_VERSION = parse_version_define(motion3d_version_header, 'MOTION3D_MAJOR_VERSION')
    MOTION3D_MINOR_VERSION = parse_version_define(motion3d_version_header, 'MOTION3D_MINOR_VERSION')
    MOTION3D_PATCH_VERSION = parse_version_define(motion3d_version_header, 'MOTION3D_PATCH_VERSION')
MOTION3D_VERSION_NUMBER = f"{MOTION3D_MAJOR_VERSION}.{MOTION3D_MINOR_VERSION}.{MOTION3D_PATCH_VERSION}"


# parse requirements
def parse_requirements(filename):
    with open(filename) as f:
        return [line.strip() for line in f.readlines()]


# configuration
setup(
    version=MOTION3D_VERSION_NUMBER,
    cmdclass={'build_ext': build_ext},
    ext_modules=ext_modules,
    install_requires=parse_requirements('requirements.txt'),
    extras_require={
        'develop': parse_requirements('requirements.develop.txt')
    },
)
