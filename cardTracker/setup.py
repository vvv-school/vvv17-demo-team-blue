import os

from setuptools import setup, find_packages

version = open(os.path.join("cardTracker", "version.txt")).read().strip()

setup( name                 = 'cardTracker',
       version              = version,
       description          = "Python/Yarp Card Tracker",
       long_description     = open("README.md").read(),
       # Get more strings from http://www.python.org/pypi?%3Aaction=list_classifiers
       classifiers          = [
         "Programming Language :: Python",
         "Topic :: Software Development :: Libraries :: Python Modules",
         ],
       keywords             = '',
       author               = 'Ingo Keller',
       author_email         = 'brutusthetschiepel@gmail.com',
       url                  = '',
       license              = 'AGPL v3',
       packages             = find_packages(exclude=['ez_setup']),
       namespace_packages   = [],
       include_package_data = True,
       zip_safe             = False,
       install_requires     = [
           'setuptools',
           # -*- Extra requirements: -*-
       ],
       entry_points         = """
       # -*- Entry points: -*-
       """,

       scripts = [
        'scripts/cardTracker',
       ]
     )
