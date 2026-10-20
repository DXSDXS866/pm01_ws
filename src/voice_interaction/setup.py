from setuptools import setup

package_name = 'voice_interaction'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['media/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice_interaction.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='PM01 Developer',
    maintainer_email='dev@example.com',
    description='PM01 语音交互节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_node = voice_interaction.voice_node:main',
        ],
    },
)
