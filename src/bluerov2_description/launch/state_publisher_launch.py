from simple_launch import SimpleLauncher
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('jsp', False)
    
    namespace = sl.arg('namespace')
    
    with sl.group(ns=namespace):

        # xacro parsing + change moving joints to fixed if no Gazebo here
        xacro_file = sl.find('bluerov2_description', 'bluerov2.xacro')
        xacro_cmd = Command(['xacro ', xacro_file, 
                            ' namespace:=', namespace,
                            ' simulation:=', sl.sim_time])
        
        # Use ParameterValue to force string type for robot_description
        sl.node('robot_state_publisher', 
                parameters=[{'robot_description': ParameterValue(xacro_cmd, value_type=str)}])

        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)
        
    return sl.launch_description()
