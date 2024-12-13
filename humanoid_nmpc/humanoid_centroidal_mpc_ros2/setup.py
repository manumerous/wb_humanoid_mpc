from setuptools import setup

package_name = "humanoid_centroidal_mpc_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    install_requires=["humanoid_common_mpc_ros2"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@email.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
