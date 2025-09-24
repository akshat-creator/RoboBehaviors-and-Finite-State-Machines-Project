from setuptools import setup

package_name = "ros_behaviors_fsm"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/fsm.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        ("share/" + package_name + "/rviz", ["rviz/default.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sailboy42",
    maintainer_email="ohimsworth@olin.edu",
    description="Behavior nodes and FSM for CompRobo warmup project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "draw_pentagon = ros_behaviors_fsm.draw_pentagon:main",
            "spin_360 = ros_behaviors_fsm.spin_360:main",
            "person_follower = ros_behaviors_fsm.person_follower:main",
            "finite_state_controller = ros_behaviors_fsm.finite_state_controller:main",
        ],
    },
)
