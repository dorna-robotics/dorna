import setuptools

with open("README.rst", "r") as fh:
    readme = fh.read()

setuptools.setup(
    name="dorna",
    version="1.3",
    author="Dorna Robotics",
    author_email="info@dorna.ai",
    description="Dorna Python API",
    long_description=readme,
    url="https://github.com/dorna-robotics/dorna",
    packages=setuptools.find_packages(),
    classifiers=[
        'Intended Audience :: Developers',
        "Programming Language :: Python :: 3.5",
        'Programming Language :: Python :: 3.6',
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
    include_package_data=True,
    zip_safe = False,
)
