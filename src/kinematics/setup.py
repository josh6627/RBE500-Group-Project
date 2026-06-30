from setuptools import find_packages, setup

package_name = "kinematics"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="josh",
    maintainer_email="josh051915@gmail.com",
    description="RBE500 Project; Nick Smith; Josh Chu",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        'console_scripts': [
            'forward = kinematics.forward_kinematics_node:main',
            'inverse = kinematics.inverse_kinematics_node:main',
        ],
    },
)
