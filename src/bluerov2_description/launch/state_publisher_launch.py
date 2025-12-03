from simple_launch import SimpleLauncher  # Utility per semplificare i launch ROS2
from launch.substitutions import Command  # Per eseguire il comando xacro in fase di launch
from launch_ros.parameter_descriptions import ParameterValue  # Forza il tipo stringa del parametro


def generate_launch_description():
    """Crea la LaunchDescription per pubblicare il robot_description e (opzionale) il joint_state.

    - Esegue il parsing del file Xacro `bluerov2.xacro` con argomenti `namespace` e `simulation`.
    - Pubblica il parametro `robot_description` per `robot_state_publisher`.
    - (Opzionale) avvia `joint_state_publisher` se richiesto da argomento.
    """
    
    sl = SimpleLauncher(use_sim_time=False)  # Qui non usiamo il tempo di simulazione
    
    # Argomenti del launch
    sl.declare_arg('namespace', default_value='bluerov2')  # Namespace del robot
    sl.declare_arg('jsp', False)  # Avvia il joint_state_publisher opzionale
    
    namespace = sl.arg('namespace')
    
    # Gruppo nominato per organizzare i nodi sotto il namespace
    with sl.group(ns=namespace):

        # Parsing del file Xacro; se simulation=False i giunti mobili possono diventare fissi
        xacro_file = sl.find('bluerov2_description', 'bluerov2.xacro')
        xacro_cmd = Command([
            'xacro ', xacro_file,
            ' namespace:=', namespace,
            ' simulation:=', sl.sim_time  # Passa true/false in base a use_sim_time
        ])
        
        # Forza il tipo stringa per il parametro robot_description
        sl.node('robot_state_publisher', 
                parameters=[{'robot_description': ParameterValue(xacro_cmd, value_type=str)}])

        # Joint State Publisher opzionale (utile per robot non simulati)
        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)
        
    # Restituisce la LaunchDescription
    return sl.launch_description()
