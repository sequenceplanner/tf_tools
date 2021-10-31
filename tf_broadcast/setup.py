from setuptools import setup

package_name = "tf_broadcast"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Endre Erős, Martin Dahl",
    author_email="endree@chalmers.se",
    maintainer="Endre Erős",
    maintainer_email="endree@chalmers.se",
    keywords=["ROS2"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="The TF Broadcast Service.",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_broadcast = tf_broadcast.tf_broadcast:main",
            "tf_broadcast_test_client = tf_broadcast.tf_broadcast_test_client:main",
        ],
    },
)
