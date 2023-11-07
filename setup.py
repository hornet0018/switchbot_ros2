from setuptools import find_packages, setup
import os

package_name = 'switchbot_ros2'

# launchディレクトリ内のファイルのリストを取得する関数
def get_launch_files():
    launch_dir = os.path.join('launch')
    launch_files = []
    # launchディレクトリが存在するか確認
    if os.path.exists(launch_dir):
        # ディレクトリ内のファイルのみをリストアップ（ディレクトリとpycファイルは除外）
        for filename in os.listdir(launch_dir):
            filepath = os.path.join(launch_dir, filename)
            if os.path.isfile(filepath) and not filename.endswith('.pyc'):
                launch_files.append(filepath)
    return launch_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', get_launch_files()),  # launchファイルを追加
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_Mater = switchbot_ros2.get_Mater:main',
        ],
    },
)
