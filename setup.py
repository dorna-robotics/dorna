import setuptools

with open("README.md", "r") as fh:
    readme = fh.read()

setuptools.setup(
    name="dorna",
    version="1.4.2",
    author="Dorna Robotics",
    author_email="info@dorna.ai",
    description="Dorna Python API",
    long_description=readme,
    long_description_content_type='text/markdown',
    url="https://dorna.ai/",
    project_urls={
        'Latest release': 'https://github.com/dorna-robotics/dorna/releases/',
    },    
    packages=setuptools.find_packages(),
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3.7',
        "Operating System :: OS Independent",
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "numpy",
        "pyserial",
    ],
    license="MIT",
    include_package_data=True,
    zip_safe = False,
)
