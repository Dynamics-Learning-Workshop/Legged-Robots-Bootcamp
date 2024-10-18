from setuptools import setup
from Cython.Build import cythonize

setup(
    name='cython_test_app',
    ext_modules=cythonize("cython_test.pyx", "cython_test2.pyx"),
)