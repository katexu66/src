from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    '''
    Launching multiple nodes simultaneously.
    Run the launch file using "ros2 launch {package} {file}"
    '''
    
    rosmav_node = Node(
        packages = 'rosmav',
        namespace='',
        executable='ros_bluerov2_interface',
        name='rosmav'
    )
    
    arming_node = Node(
        packages = 'rosmav',
        namespace='',
        executable='arming',
        name='arming'
    )
    
    dancing_node = Node(
        packages = 'rosmav',
        namespace='',
        executable='movement',
        name='dancing'
    )
    
    return LaunchDescription(arming_node, dancing_node) #brackets?

generate_launch_description()

#need to move this file into launch folder in src
#may need to add ros2launch to package.xml exec_depend dependency