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
sys.path.append(os.path.join(os.path.dirname(__file__), "../../../"))
sys.path.append(os.path.join(os.path.dirname(__file__), "../../../commonroad_reach/"))


# -- Project information -----------------------------------------------------

project = "CommonRoad-Reach"
copyright = "2022, Technical University of Munich, Cyber-Physical Systems Group"
author = "Edmond Irani Liu, Gerald Würsching, Moritz Klischat, Matthias Althoff"

# The full version, including alpha/beta/rc tags
release = "2022.3"


# -- General configuration ---------------------------------------------------
# Sphinx considers the files with these suffices as sources.
source_suffix = ['.rst', '.md']
# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ["sphinx.ext.autodoc", 
              "sphinx.ext.viewcode",
              "sphinx.ext.napoleon",
              'sphinx.ext.inheritance_diagram',
              "sphinx.ext.todo",
              'sphinx_autodoc_typehints',    
              "m2r2"
              ]
# order of displaying the members in an automodule/autoclass
autodoc_member_order = "bysource"

# show todo items
todo_include_todos = True

autodoc_typehints = "both"

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
