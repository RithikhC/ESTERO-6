from glob import glob

from setuptools import setup

package_name = "bayani_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Bayani-6 autonomous estero guardian",
    entry_points={
        "console_scripts": [
            "anomaly_detector = bayani_core.anomaly_detector:main",
            "mission_master = bayani_core.mission_master:main",
            "arm_sampler = bayani_core.arm_sampler:main",
            "make_heatmap = bayani_core.make_heatmap:main",
        ],
    },
)
