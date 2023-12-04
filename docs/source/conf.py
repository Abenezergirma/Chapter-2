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
