# Motion3d Documentation

The following sections describe how to build the C++ and Python documentation pages and which dependencies are required.
In case all dependencies are available, you can also use the `build.py` script for building both documentations automatically.


## Doxygen C++ Documentation

The C++ documentation is created using Doxygen (https://doxygen.nl).
Further, MathJax is required for rendering LaTeX formulas.

After installing Doxygen and LaTeX, run the following command from the `doxygen` directory:

```bash
doxygen Doxyfile
```

The documentation main page is then located at `doxygen/html/index.html`. 


## Sphinx Python Documentation

The Python documentation is created using Sphinx (https://www.sphinx-doc.org).
Since the Python library is fully binded, the documentation is created automatically and linked to the respective C++ documentation created by Doxygen using Breathe (https://breathe.readthedocs.io).
Thus, the Doxygen documentation must be created and motion3d must be installed before creating the Sphinx documentation.

All requirements for the Python documentation can be installed using:

```bash
bash> python3 -m pip install .[develop]
zsh>  python3 -m pip install .\[develop\]
```

After building the doxygen documentation and installing all requirements, run the following command from the `sphinx` directory:

```bash
make html
```

The documentation main page is then located at `sphinx/build/html/index.html`. 
