from simple_launch import SimpleLauncher, GazeboBridge  # Utility per semplificare launch ROS2 + Gazebo

sl = SimpleLauncher(use_sim_time = True)  # Abilita il tempo di simulazione (use_sim_time)

sl.declare_arg('namespace', default_value='bluerov2')        # Namespace del robot
sl.declare_arg('ground_truth', default_value=True)            # Pubblica odometria ground truth (pose e tf)
sl.declare_arg('sliders', default_value=False)                # Avvia nodo slider per comandi manuali
sl.declare_arg('camera', True)                                # Se true abilita il bridge immagine camera
sl.declare_arg('gazebo_world_name', 'none')                   # Nome del mondo Gazebo (se diverso da default)

# Pose iniziale del modello nella simulazione
sl.declare_gazebo_axes(x=1., y=0., z=1., roll=0., pitch=0., yaw=0.)


def launch_setup():
    """Configura e crea la LaunchDescription per caricare il modello BlueROV2 e i bridge.
    - Spawna il modello tramite URDF nel mondo corrente.
    - Crea bridge ROS2 <-> Gazebo per sensori, attuatori e ground truth.
    - Pubblica TF della posa del robot.
    """
    
    ns = sl.arg('namespace')  # Recupera il namespace richiesto

    # Imposta il nome del mondo Gazebo se fornito
    if sl.arg('gazebo_world_name') != 'none':
        GazeboBridge.set_world_name(sl.arg('gazebo_world_name'))
    
    # Include il robot_state_publisher per pubblicare il TF dell'URDF
    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': ns, 'use_sim_time': sl.sim_time})
               
    with sl.group(ns=ns):
                    
        # Spawna il modello URDF nel mondo, usando la pose dichiarata
        sl.spawn_gz_model(ns, spawn_args=sl.gazebo_axes_args())
            
        # ROS-Gz bridges
        bridges = []  # Lista dei bridge da creare
        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        # Bridge per lo stato dei giunti (Gazebo -> ROS)
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # Bridge della pose (ground truth) singola del modello
        bridges.append(GazeboBridge(f'/model/{ns}/pose',
                        'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # Bridge odometria + pubblicazione TF se ground_truth richiesto
        if sl.arg('ground_truth'):
            bridges.append(GazeboBridge(f'/model/{ns}/odometry',
                        'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros,
                        'gz.msgs.Odometry'))
            sl.node('pose_to_tf', parameters={'child_frame': ns + '/base_link'},
                output='screen')  # Pubblica TF base_link
        else:
            # Pubblica frame alternativo base_link_gt se non si usa ground_truth
            sl.node('pose_to_tf', parameters={'child_frame': ns + '/base_link_gt'})

        # Bridge delle due IMU (mpu, lsm) Gazebo -> ROS
        for imu in ('mpu', 'lsm'):
            bridges.append(GazeboBridge(f'{ns}/{imu}',
                                        imu, 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        # Sonar: pubblicato come LaserScan + PointCloud2
        bridges.append(GazeboBridge(f'{ns}/sonar', 'sonar', 'sensor_msgs/LaserScan', GazeboBridge.gz2ros))
        bridges.append(GazeboBridge(f'{ns}/sonar/points', 'cloud', 'sensor_msgs/PointCloud2', GazeboBridge.gz2ros))

        # Immagine camera (abilitata se arg camera=True)
        if sl.arg('camera'):
            bridges.append(GazeboBridge(f'{ns}/image', 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))
        
        # Comando di posizione giunto tilt (ROS -> Gazebo)
        bridges.append(GazeboBridge(f'/model/{ns}/joint/tilt/0/cmd_pos',
                        'cmd_tilt', 'std_msgs/Float64', GazeboBridge.ros2gz))
        
        # Comandi dei 6 thruster (ROS -> Gazebo)
        for thr in range(1, 7):
            thruster = f'thruster{thr}'
            gz_thr_topic = f'/{ns}/{thruster}/cmd'
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{thruster}', 'std_msgs/Float64', GazeboBridge.ros2gz))

        # Crea effettivamente tutti i bridge definiti
        sl.create_gz_bridge(bridges)

        # Nodo opzionale di interfaccia slider per inviare comandi manuali
        if sl.arg('sliders'):
            sl.node('slider_publisher', arguments=[sl.find('bluerov2_description', 'manual.yaml')])
    
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)  # Entry point per ROS2
