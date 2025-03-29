from setuptools import setup, find_packages

setup(
    name='calib_board',
    version='0.1',
    packages=find_packages(),
    install_requires=[],
    entry_points={
        'console_scripts': [
            'calibrate-extrinsics = calib_board.scripts.run:main'
        ],
    },
)