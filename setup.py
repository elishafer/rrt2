from distutils.core import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize("path_planning/primatives.pyx"),
)