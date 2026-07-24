from setuptools import find_packages, setup


package_name = "ugv_mvp_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xcc",
    maintainer_email="xcxc@nuaa.com",
    description="Development-only standard-message fixtures for the UGV MVP.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "path_fixture_node = ugv_mvp_tools.path_fixture_node:main",
            "raw_odom_fixture_node = ugv_mvp_tools.raw_odom_fixture_node:main",
            "pointcloud_fixture_node = ugv_mvp_tools.pointcloud_fixture_node:main",
            "next_waypoint_fixture_node = ugv_mvp_tools.next_waypoint_fixture_node:main",
            "nominal_cmd_fixture_node = ugv_mvp_tools.nominal_cmd_fixture_node:main",
            "static_tf_fixture_node = ugv_mvp_tools.static_tf_fixture_node:main",
        ],
    },
)
