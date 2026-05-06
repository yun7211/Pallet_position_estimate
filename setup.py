try:
    from catkin_pkg.python_setup import generate_distutils_setup

    setup_args = generate_distutils_setup(
        packages=["pallet_pose_estimation"],
        package_dir={"": "src"},
    )
    from distutils.core import setup
except ImportError:
    from setuptools import find_packages, setup

    setup_args = {
        "name": "pallet_pose_estimation",
        "version": "0.1.0",
        "package_dir": {"": "src"},
        "packages": find_packages("src"),
    }


setup(**setup_args)
