from setuptools import setup
from setuptools import Extension
from Cython.Build import cythonize
cy_opts = {'compiler_directives': {'language_level': '3'}}

ext_mods = [Extension(
    'wrapper_module_0', ['wrapper_module_0.pyx', 'wrapped_code_0.c'],
    include_dirs=[],
    library_dirs=[],
    libraries=[],
    extra_compile_args=['-std=c99'],
    extra_link_args=[]
)]
setup(ext_modules=cythonize(ext_mods, **cy_opts))
