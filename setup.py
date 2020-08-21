import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="rbw",
    version="0.1.0",
    author="Mario Belledonne",
    author_email="mbelledonne@gmail.com",
    description="Simulation environment of rigid body worlds",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages = ["rbw"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires = [
        'numpy',
        'scipy',
        'networkx',
        'pybullet',
        'pyquaternion'
    ]
)
