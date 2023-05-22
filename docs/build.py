#!/usr/bin/env python3
import distutils.spawn
import importlib
import os.path as osp
import subprocess
import sys


# utils
script_dir = osp.dirname(osp.realpath(__file__))

def exit_with_error(msg):
    print(msg, file=sys.stderr)
    sys.exit(1)


# check C++ dependencies
cpp_dependencies = ['doxygen', 'latex', 'make']
for dependency in cpp_dependencies:
    if distutils.spawn.find_executable(dependency) is None:
        exit_with_error(f"Error: required dependency '{dependency}' not found.")


# check Python dependencies
python_packages = ['breathe', 'sphinx', 'sphinx_rtd_theme', 'motion3d']
for package in python_packages:
    try:
        importlib.import_module(package)
    except ModuleNotFoundError:
        exit_with_error(f"Error: required python package '{package}' not found.\n"
                         "Did you install the package with the [develop] option?")

        

# build C++ documentation using Doxygen
print("##################################################")
print("###  Building C++ Documentation using Doxygen  ###")
print("##################################################")
doxygen_dir = osp.join(script_dir, 'doxygen')
doxygen = subprocess.Popen(['doxygen', 'Doxyfile'], cwd=doxygen_dir)
doxygen.wait()
if doxygen.returncode != 0:
    exit_with_error(f"An error occured while building the C++ documentation.")


# build Python documentation using Sphinx
print("\n#################################################")
print("### Building Full Documentation using Sphinx  ###")
print("#################################################")
sphinx_dir = osp.join(script_dir, 'sphinx')
sphinx = subprocess.Popen(['make', 'html'], cwd=sphinx_dir)
sphinx.wait()
if sphinx.returncode != 0:
    exit_with_error(f"An error occured while building the Python documentation.")


# print main pages
print("\nSucessfully built the C++ and Python documentation:")
print(f" - Doxygen main page: {osp.join(doxygen_dir, 'build', 'html', 'index.html')}")
print(f" - Sphinx main page:  {osp.join(sphinx_dir, 'build', 'html', 'index.html')}")
