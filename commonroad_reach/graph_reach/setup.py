from distutils.core import setup
from Cython.Build import cythonize

# TODO: move to main setup.py
ext_options = {"compiler_directives": {"profile": True}, "annotate": True}
setup(
    ext_modules = cythonize("util_reach.pyx", **ext_options)
)
