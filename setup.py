from distutils.core import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize("path_planning/control_space_env.pyx"),
)
setup(
    ext_modules=cythonize("path_planning/rrt_tree.pyx"),
)
setup(
    ext_modules=cythonize("path_planning/primatives.pyx"),
)
setup(
    ext_modules=cythonize("path_planning/rrt2.pyx"),
)
