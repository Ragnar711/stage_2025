import socket
import json
import os
from ament_index_python.packages import get_package_share_directory  # type: ignore


def read_hosts_file():
    land_drone_directory = get_package_share_directory("land_drone")
    hosts_file = os.path.join("hosts.json")

    script_dir_json = land_drone_directory + "/" + hosts_file

    with open(script_dir_json, "r") as json_file:
        address_list = json.load(json_file)

    return address_list


def read_config_file(config_file):
    try:
        with open(config_file, "r") as f:
            config = json.load(f)
            return config
    except FileNotFoundError:
        print(f"Erreur: Le fichier {config_file} n'a pas été trouvé.")
        return None
    except json.JSONDecodeError as e:
        print(f"Erreur de décodage JSON dans {config_file}: {e}")
        return None


def get_name_robot():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip_address = s.getsockname()[0]
    address_list = read_hosts_file()
    name = ""
    neighbors = []
    for key, drone in address_list.items():
        if ip_address == key:
            name = drone
        else:
            neighbors.append(drone)
    return name, neighbors
