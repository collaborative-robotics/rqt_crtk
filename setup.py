from setuptools import find_packages, setup

setup(name="rqt_crtk", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/rqt_crtk"]),
                  ("share/ament_index/resource_index/rqt_gui__pluginlib__plugin",
                   ["resource/rqt_gui__pluginlib__plugin/rqt_crtk"]),
                  ("share/rqt_crtk", ["package.xml", "plugin.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="dVRK maintainers", maintainer_email="support@intusurg.com",
      description="Generic ROS 2 rqt plugins for CRTK devices.", license="BSD-3-Clause")
