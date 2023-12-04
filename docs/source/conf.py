# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Chapter-2'
copyright = '2023, Abenezer Taye'
author = 'Abenezer Taye'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# Extension for auto-api documentation
extensions.append('autoapi.extension')
autoapi_type = 'python'
autoapi_dirs = ['../Chapter-2']

# Extension for adding notebooks (and markdown) to the documentation:
extensions.append('myst_nb')
# the above already registers the following. Re-adding it will cause it to
# re-register the .md extension for parsing by myst, giving an error.
# for embedding readme.md into main documentation
# extensions.append('myst_parser')

myst_enable_extensions = [
    "amsmath",
    "colon_fence",
    "deflist",
    "dollarmath",
    "html_image",
]
myst_url_schemes = ("http", "https", "mailto")
nb_execution_excludepatterns = ()
nb_execution_timeout = 180




# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
html_theme_options = {
    'github_user': 'abenezergirma',
    'github_repo': 'Chapter-2',
    'github_banner': True,
    'github_button': True,
    'github_type': 'star',
    'fixed_sidebar': True,
    'logo': 'images/chapter2_icon.png',
    'description': 'Pre‑departure Flight Planning and Risk Assessment under Battery Constraint',
    'logo_name': True
}
