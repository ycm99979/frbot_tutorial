import os 

#다른 패키지에 존재하는 파일들을 가져옴
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
#from nav2_common.launch import 아마 통합 런치파일 같은데 모름

def generate_launch_description():
    package_path = get_package_share_directory('frbot_navigation')
    
    #파라미터 파일을 보통 config이라고 명명한 패키지에 넣는다 
    config_path = os.path.join(package_path, 'config')
    
    #AMCL 설정 파일 경로
    amcl_file_path = amcl.yaml
    
    #rviz만 사용했을 때 사용할 map 파일 (이때 형식은 .pgm 으로 2d_occpancy_grid_map 이다)
    map_path = os.path.join(package_path, 'map')
    map_file_path = 'pknu_3rd_0823' #아직 없음 (11/12 기준)
    
    
    
    