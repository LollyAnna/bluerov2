from simple_launch import SimpleLauncher, GazeboBridge  # Import delle utility per semplificare i launch ROS2 + Gazebo


def generate_launch_description():
    """Genera la Launch Description per avviare il mondo ocean e (opzionale) il modello BlueROV2.

    Argomenti dichiarabili:
    - gui: se True avvia Gazebo con interfaccia grafica, altrimenti headless (-s)
    - spawn: se True include il launch di upload del robot (caricamento modello)
    """
    sl = SimpleLauncher()

    # Dichiarazione degli argomenti di configurazione
    sl.declare_arg('gui', default_value=True)      # Mostra GUI di Gazebo
    sl.declare_arg('spawn', default_value=True)    # Spawna il modello del BlueROV2

    # Gruppo eseguito se gui=True: lancio del mondo con rendering grafico (-r)
    with sl.group(if_arg='gui'):
        sl.gz_launch(sl.find('bluerov2_description', 'demo_world.sdf'), "-r")

    # Gruppo alternativo se gui=False: modalità server/headless (-s)
    with sl.group(unless_arg='gui'):
        sl.gz_launch(sl.find('bluerov2_description', 'demo_world.sdf'), "-r -s")

    # Definizione dei bridge ROS2 <-> Gazebo: clock e corrente marina
    bridges = [
        GazeboBridge.clock(),  # Bridge del tempo di simulazione
        GazeboBridge('/ocean_current', '/current', 'geometry_msgs/Vector3', GazeboBridge.ros2gz)
        # Converte il topic ROS2 '/current' verso Gazebo '/ocean_current'
    ]
    sl.create_gz_bridge(bridges)

    # Inclusione del launch per caricare il robot se richiesto
    with sl.group(if_arg='spawn'):
        sl.include('bluerov2_description', 'upload_bluerov2_launch.py')

    # Restituisce l'oggetto LaunchDescription costruito
    return sl.launch_description()
