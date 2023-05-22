Development
===========


C++
---

For building the C++ unit tests with coverage reports and activated clang-tidy use:

.. code-block:: bash

    mkdir build && cd build
    cmake -DCOVERAGE=ON -DTESTS=ON -DTIDY=ON ..
    make

After running the unit tests with ``make test``, you can get a coverage summary by executing ``gcovr --print-summary`` from the project root directory.


Python
------

Use the following command to install the library with all required development dependencies:

.. code-block:: bash

    bash> python3 -m pip install .[develop]
    zsh>  python3 -m pip install .\[develop\]

You can now run the style guide tests with ``flake8`` and the Python unit tests with ``pytest``.
