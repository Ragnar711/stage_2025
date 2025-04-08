import socket
import json
import os
from ament_index_python.packages import get_package_share_directory  # type: ignore
import pandas as pd  # type: ignore


def read_hosts_file():
    land_drone_directory = get_package_share_directory("land_drone")
    hosts_file = os.path.join("hosts.json")

    script_dir_json = land_drone_directory + "/" + hosts_file

    with open(script_dir_json, "r") as json_file:
        address_list = json.load(json_file)

    return address_list


# def read_config_file(config_file):
#     try:
#         with open(config_file, 'r') as f:
#             config = json.load(f)
#             return config
#     except FileNotFoundError:
#         print(f"Erreur: Le fichier {config_file} n'a pas été trouvé.")
#         return None
#     except json.JSONDecodeError as e:
#         print(f"Erreur de décodage JSON dans {config_file}: {e}")
#         return None


def read_config_file(config_file):
    # JSON file
    f = open(config_file, "r")

    # Reading from file
    config = json.loads(f.read())

    # Iterating through the json list
    for i in config["experiences"]:
        print(i)

    # Closing file
    f.close()
    return config


def csv_to_df(path, file, index_col=None):
    # Import data from .csv file
    fname = os.path.join(path, file)
    df = pd.read_csv(fname, index_col=index_col)
    return df


def read_xml_file(folder_path, filename):
    # Return xml file in string form
    filename_path = os.path.join(folder_path, filename)
    with open(filename_path, "r") as f:
        xml = f.read()
    return xml


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
