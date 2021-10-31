from setuptools import setup

package_name = "tf_lookup"

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
    author_email="endree@chalmers.se, martin.dahl@chalmers.se",
    maintainer="Endre Erős",
    maintainer_email="endree@chalmers.se",
    keywords=["ROS2"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="The TF Lookup Package.",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_lookup = tf_lookup.tf_lookup:main",
            "tf_broadcast = tf_lookup.tf_broadcast:main",
            "tf_lookup_client = tf_lookup.tf_lookup_client:main",
            "tf_broadcast_client = tf_lookup.tf_broadcast_client:main",
        ],
    },
)
