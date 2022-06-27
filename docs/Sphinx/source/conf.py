# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath("../../../"))
sys.path.insert(0, os.path.abspath('../../../commonroad_reach/data_structure/'))
sys.path.insert(0, os.path.abspath('../../../commonroad_reach/data_structure/reach/'))
sys.path.insert(0, os.path.abspath('../../../commonroad_reach/utility/'))


# -- Project information -----------------------------------------------------

project = "CommonRoad-Reach"
copyright = "2022, Technical University of Munich, Cyber-Physical Systems Group"
author = "Edmond Irani Liu, Gerald Würsching, Moritz Klishcat, Matthias Althoff"

# The full version, including alpha/beta/rc tags
release = "1.0.0"


# -- General configuration ---------------------------------------------------
# Sphinx considers the files with these suffices as sources.
source_suffix = ['.rst', '.md']
# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ["sphinx.ext.autodoc", 
              "sphinx.ext.intersphinx",
              "sphinx.ext.viewcode",
              "sphinx.ext.napoleon",
              "m2r2",
              "sphinxcontrib.apidoc",
              "sphinx.ext.todo"
              ]
# configure sphinxcontrib.apidoc
apidoc_module_dir = '../../../commonroad_reach/'
apidoc_output_dir = './'
apidoc_excluded_paths = ['tests/']
apidoc_separate_modules = False
# apidoc_module_first = True

# show todo items
todo_include_todos = True

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    # Toc options
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

html_logo = '_static/commonroad_white150.png'
