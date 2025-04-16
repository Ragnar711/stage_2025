import socket
import json
import os
import pandas as pd  # type: ignore
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def read_hosts_file():
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        hosts_file_path = os.path.abspath(
            os.path.join(script_dir, "..", "..", "config", "hosts.json")
        )

        logging.debug(f"Attempting to read hosts file from: {hosts_file_path}")

        if not os.path.exists(hosts_file_path):
            logging.error(f"Hosts file not found at calculated path: {hosts_file_path}")
            return None

        with open(hosts_file_path, "r") as json_file:
            address_list = json.load(json_file)
        return address_list

    except FileNotFoundError:
        logging.error(f"Error: The file {hosts_file_path} was not found.")
        return None
    except json.JSONDecodeError as e:
        logging.error(f"Error decoding JSON from {hosts_file_path}: {e}")
        return None
    except Exception as e:
        logging.error(f"An unexpected error occurred while reading hosts file: {e}")
        return None


def read_config_file(config_file_path):
    try:
        with open(config_file_path, "r") as f:
            config = json.load(f)
            return config
    except FileNotFoundError:
        logging.error(
            f"Error: The configuration file {config_file_path} was not found."
        )
        return None
    except json.JSONDecodeError as e:
        logging.error(f"Error decoding JSON in {config_file_path}: {e}")
        return None
    except Exception as e:
        logging.error(
            f"An unexpected error occurred while reading config file {config_file_path}: {e}"
        )
        return None


# def read_config_file(config_file):
#     # JSON file
#     f = open(config_file, "r")

#     # Reading from file
#     config = json.loads(f.read())

#     # Iterating through the json list
#     for i in config["experiences"]:
#         print(i)

#     # Closing file
#     f.close()
#     return config


def csv_to_df(path, file, index_col=None):
    try:
        fname = os.path.join(path, file)
        df = pd.read_csv(fname, index_col=index_col)
        return df
    except FileNotFoundError:
        logging.error(f"Error: CSV file not found at {os.path.join(path, file)}")
        return None
    except Exception as e:
        logging.error(
            f"An error occurred while reading CSV {os.path.join(path, file)}: {e}"
        )
        return None


def read_xml_file(folder_path, filename):
    try:
        filename_path = os.path.join(folder_path, filename)
        with open(filename_path, "r") as f:
            xml_content = f.read()
        return xml_content
    except FileNotFoundError:
        logging.error(
            f"Error: XML file not found at {os.path.join(folder_path, filename)}"
        )
        return None
    except Exception as e:
        logging.error(
            f"An error occurred while reading XML {os.path.join(folder_path, filename)}: {e}"
        )
        return None


def get_name_robot():
    name = ""
    neighbors = []
    ip_address = None

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(0.1)  # Prevent long hang if 8.8.8.8 is unreachable
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
    except socket.error as e:
        logging.error(
            f"Could not determine local IP address: {e}. Cannot determine robot name."
        )
        return "", []  # Return default empty values

    address_list = read_hosts_file()

    if address_list is None:
        logging.error(
            "Failed to read hosts file. Cannot determine robot name or neighbors."
        )
        return "", []  # Return default empty values if hosts file reading failed

    found_self = False
    for key, drone in address_list.items():
        if ip_address == key:
            name = drone
            found_self = True
        else:
            neighbors.append(drone)

    if not found_self:
        logging.warning(f"Local IP {ip_address} not found in hosts file.")

    return name, neighbors
