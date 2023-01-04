#!/usr/bin/python3

from setuptools import setup
from setuptools.extension import Extension
from Cython.Build import cythonize

ext_modules = [  # List of modules to compile
	Extension("tbouncer_cy",
		sources=["tbouncer_cy.pyx", "../tbouncer.c"],
		include_dirs=['..'],
		define_macros=[
			('LIBAVR_TEST_BUILD', None),
			('TBOUNCER_DISABLE_PORT_C', None),
			('TBOUNCER_DISABLE_PORT_D', None),
			('F_CPU', '16000000L'),
			('SCHED_CLOCK_PRESCALE_LOG', '5')])]

setup(
	name='libavr tests',
	ext_modules=cythonize(
		ext_modules,
		compiler_directives={ 'language_level' : 3 },  # Python 3 styled Cython
		annotate=True),  # Output HTML files that show the Cython-C mapping.
	zip_safe=False)
