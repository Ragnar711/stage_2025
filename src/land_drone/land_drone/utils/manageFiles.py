# src/land_drone/land_drone/utils/manageFiles.py (Client with Monitor)

import socket
import json
import os
import pandas as pd  # type: ignore
import requests
import logging
import time
import threading
from sseclient import SSEClient  # pip install sseclient-py

# --- Client Configuration ---
HOSTS_SERVER_URL = os.environ.get("HOSTS_SERVER_URL", "http://localhost:5000")
ROBOT_NAME = os.environ.get(
    "ROBOT_NAME", f"robot_{int(time.time())}"
)  # Default to somewhat unique name
REQUEST_TIMEOUT = 10  # Seconds for normal HTTP requests
HEARTBEAT_INTERVAL = 30  # Seconds between sending heartbeats
SSE_RECONNECT_DELAY = 5  # Seconds before attempting SSE reconnect on error
# --- End Client Configuration ---

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - [%(threadName)s] - %(message)s",
)
logger = logging.getLogger(f"manageFiles.{ROBOT_NAME}")

# --- Global variable to hold the monitor instance ---
# This allows different parts of the robot code to access the same monitor
_host_monitor_instance = None
_monitor_lock = threading.Lock()
# ---


def get_local_ip_address():
    """Tries to determine the local IP address"""
    s = None
    ip_address = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
    except socket.error as e:
        logger.error(f"Could not determine local IP address via socket: {e}")
        return None
    finally:
        if s:
            s.close()
    return ip_address


class HostMonitor:
    """Manages connection, registration, heartbeats, and neighbor updates."""

    def __init__(self, server_url, robot_name, heartbeat_interval, req_timeout):
        self.server_url = server_url
        self.robot_name = robot_name
        self.heartbeat_interval = heartbeat_interval
        self.req_timeout = req_timeout
        self.robot_ip = None

        self._neighbors = {}  # Stores current {ip: name} of neighbors
        self._neighbors_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sse_thread = None
        self._heartbeat_thread = None
        self.session = (
            requests.Session()
        )  # Use a session for potential connection reuse

    def _update_neighbors(self, all_hosts):
        """Safely updates the internal neighbor list."""
        with self._neighbors_lock:
            self._neighbors.clear()
            for ip, name in all_hosts.items():
                if name != self.robot_name:
                    self._neighbors[ip] = name
            logger.info(f"Neighbors updated: {list(self._neighbors.values())}")
            # TODO: Add a callback mechanism here if needed
            # e.g., if self.on_update_callback: self.on_update_callback(self.get_neighbors())

    def register(self):
        """Registers this robot with the server and gets initial neighbors."""
        self.robot_ip = get_local_ip_address()
        if not self.robot_ip:
            logger.critical("Failed to determine local IP. Cannot register.")
            return False  # Indicate failure

        register_url = f"{self.server_url}/register"
        payload = {"name": self.robot_name, "ip": self.robot_ip}
        try:
            logger.info(f"Attempting initial registration: {payload}")
            response = self.session.post(
                register_url, json=payload, timeout=self.req_timeout
            )
            response.raise_for_status()
            response_data = response.json()
            logger.info(f"Registration successful: {response_data.get('message', '')}")
            # Initial neighbors list from registration response
            self._update_neighbors(response_data.get("neighbors", {}))
            return True  # Indicate success
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to register with server at {register_url}: {e}")
            if e.response is not None:
                try:
                    logger.error(f"Server response: {e.response.json()}")
                except json.JSONDecodeError:
                    logger.error(f"Server response (non-JSON): {e.response.text}")
            return False  # Indicate failure

    def _listen_for_updates(self):
        """Background thread function to listen for SSE updates."""
        sse_url = f"{self.server_url}/events"
        logger.info(f"SSE listener thread started, connecting to {sse_url}")
        while not self._stop_event.is_set():
            try:
                # Use the shared session
                response = self.session.get(
                    sse_url, stream=True, timeout=self.req_timeout + 5
                )  # Slightly longer timeout for stream
                response.raise_for_status()
                client = SSEClient(response)  # Wrap the response stream
                logger.info(f"SSE connection established to {sse_url}")

                for event in client.events():  # This blocks until an event arrives
                    if self._stop_event.is_set():
                        break  # Exit if stop signal received

                    logger.debug(
                        f"SSE event received: type={event.event}, data={event.data}"
                    )
                    if event.data:
                        try:
                            all_hosts = json.loads(event.data)
                            if isinstance(all_hosts, dict):
                                self._update_neighbors(all_hosts)
                            else:
                                logger.warning(
                                    f"Received non-dict data in SSE event: {event.data}"
                                )
                        except json.JSONDecodeError:
                            logger.error(
                                f"Failed to decode JSON from SSE event: {event.data}"
                            )
                        except Exception as e:
                            logger.error(
                                f"Error processing SSE event: {e}", exc_info=True
                            )

            except requests.exceptions.ConnectionError as e:
                logger.error(
                    f"SSE connection error to {sse_url}: {e}. Retrying in {SSE_RECONNECT_DELAY}s..."
                )
            except requests.exceptions.HTTPError as e:
                logger.error(
                    f"SSE HTTP error from {sse_url}: {e.response.status_code}. Retrying in {SSE_RECONNECT_DELAY}s..."
                )
                if e.response is not None:
                    logger.error(f"Server response: {e.response.text}")
            except requests.exceptions.RequestException as e:
                logger.error(
                    f"SSE request error for {sse_url}: {e}. Retrying in {SSE_RECONNECT_DELAY}s..."
                )
            except Exception as e:
                # Catch potential errors within SSEClient or loops
                logger.error(
                    f"Unexpected error in SSE listener loop: {e}", exc_info=True
                )
                logger.info(f"Retrying SSE connection in {SSE_RECONNECT_DELAY}s...")

            # Wait before retrying or if loop finished unexpectedly
            if not self._stop_event.is_set():
                self._stop_event.wait(SSE_RECONNECT_DELAY)  # Wait before retry

        logger.info("SSE listener thread finished.")
        if hasattr(client, "close"):
            client.close()  # Close underlying connection if possible

    def _send_heartbeats(self):
        """Background thread function to send periodic heartbeats."""
        heartbeat_url = f"{self.server_url}/heartbeat"
        payload = {"name": self.robot_name}
        logger.info(
            f"Heartbeat thread started. Sending to {heartbeat_url} every {self.heartbeat_interval}s"
        )

        while not self._stop_event.wait(
            self.heartbeat_interval
        ):  # Wait for interval or stop signal
            try:
                logger.debug(f"Sending heartbeat: {payload}")
                response = self.session.post(
                    heartbeat_url, json=payload, timeout=self.req_timeout
                )
                response.raise_for_status()  # Check for 4xx/5xx errors
                logger.debug(f"Heartbeat acknowledged by server.")
            except requests.exceptions.RequestException as e:
                logger.warning(f"Failed to send heartbeat to {heartbeat_url}: {e}")
                # Don't necessarily stop the whole monitor on heartbeat failure, maybe server is temp down
            except Exception as e:
                logger.error(f"Unexpected error sending heartbeat: {e}", exc_info=True)

        logger.info("Heartbeat thread finished.")

    def start(self):
        """Registers and starts background threads for SSE and heartbeats."""
        if not self.register():  # Attempt initial registration
            logger.error(
                "Initial registration failed. Monitor will not start background tasks."
            )
            return  # Don't start threads if registration failed

        if self._sse_thread is None or not self._sse_thread.is_alive():
            self._stop_event.clear()  # Ensure stop is not set
            self._sse_thread = threading.Thread(
                target=self._listen_for_updates,
                daemon=True,
                name=f"{self.robot_name}-SSE",
            )
            self._sse_thread.start()
        else:
            logger.warning("SSE thread already running.")

        if self._heartbeat_thread is None or not self._heartbeat_thread.is_alive():
            # Ensure stop is clear if restarting only this thread (though usually started together)
            # self._stop_event.clear() # Probably not needed if start() is called once
            self._heartbeat_thread = threading.Thread(
                target=self._send_heartbeats, daemon=True, name=f"{self.robot_name}-HB"
            )
            self._heartbeat_thread.start()
        else:
            logger.warning("Heartbeat thread already running.")

    def stop(self):
        """Stops background threads and explicitly unregisters from the server."""
        logger.info("Stopping Host Monitor...")
        self._stop_event.set()  # Signal threads to stop

        # Attempt explicit unregistration
        unregister_url = f"{self.server_url}/unregister/{self.robot_name}"
        try:
            logger.info(f"Sending explicit unregister request to {unregister_url}")
            response = self.session.delete(unregister_url, timeout=self.req_timeout)
            if response.ok:
                logger.info(
                    f"Unregistration request successful: {response.json().get('message', '')}"
                )
            else:
                logger.warning(
                    f"Unregistration request failed: {response.status_code} - {response.text}"
                )
        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to send unregister request: {e}")

        # Wait briefly for threads to finish
        if self._sse_thread and self._sse_thread.is_alive():
            logger.debug("Waiting for SSE thread to join...")
            self._sse_thread.join(timeout=2)
            if self._sse_thread.is_alive():
                logger.warning("SSE thread did not join cleanly.")

        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            logger.debug("Waiting for Heartbeat thread to join...")
            self._heartbeat_thread.join(
                timeout=1
            )  # Should exit quickly after event set
            if self._heartbeat_thread.is_alive():
                logger.warning("Heartbeat thread did not join cleanly.")

        self.session.close()  # Close the requests session
        logger.info("Host Monitor stopped.")

    def get_neighbors(self):
        """Returns the current list of neighbor names."""
        with self._neighbors_lock:
            # Return a copy of the values (names)
            return list(self._neighbors.values())

    def get_neighbors_with_ips(self):
        """Returns the current dictionary of {ip: name} for neighbors."""
        with self._neighbors_lock:
            # Return a copy of the dictionary
            return self._neighbors.copy()


# --- Public Functions using the Monitor ---


def start_host_monitor():
    """Initializes and starts the singleton HostMonitor instance."""
    global _host_monitor_instance
    with _monitor_lock:
        if _host_monitor_instance is None:
            logger.info("Creating and starting Host Monitor instance...")
            monitor = HostMonitor(
                server_url=HOSTS_SERVER_URL,
                robot_name=ROBOT_NAME,
                heartbeat_interval=HEARTBEAT_INTERVAL,
                req_timeout=REQUEST_TIMEOUT,
            )
            monitor.start()  # This attempts registration and starts threads
            _host_monitor_instance = monitor
            # Add a cleanup function to be called at Python exit
            import atexit

            atexit.register(stop_host_monitor)
        else:
            logger.debug("Host Monitor instance already exists.")
        return _host_monitor_instance


def stop_host_monitor():
    """Stops the singleton HostMonitor instance if it exists."""
    global _host_monitor_instance
    with _monitor_lock:
        if _host_monitor_instance is not None:
            logger.info("Stopping singleton Host Monitor instance...")
            _host_monitor_instance.stop()
            _host_monitor_instance = None
        else:
            logger.debug("Stop called but Host Monitor instance does not exist.")


def get_robot_name():
    """Gets the configured robot name."""
    if ROBOT_NAME.startswith("robot_"):
        logger.warning("ROBOT_NAME environment variable not set, using default.")
    return ROBOT_NAME


def get_current_neighbors():
    """
    Gets the latest list of neighbor names from the monitor.
    Starts the monitor if it hasn't been started yet.
    """
    monitor = start_host_monitor()  # Ensure monitor is running
    if monitor:
        return monitor.get_neighbors()
    else:
        logger.error(
            "Host monitor could not be started. Returning empty neighbors list."
        )
        return []  # Return empty list if monitor failed to start


def get_current_neighbors_with_ips():
    """
    Gets the latest {ip: name} dictionary for neighbors from the monitor.
    Starts the monitor if it hasn't been started yet.
    """
    monitor = start_host_monitor()  # Ensure monitor is running
    if monitor:
        return monitor.get_neighbors_with_ips()
    else:
        logger.error(
            "Host monitor could not be started. Returning empty neighbors dict."
        )
        return {}  # Return empty dict if monitor failed to start


# --- Other utility functions remain the same (no changes needed) ---


def read_config_file(config_file_path):
    try:
        with open(config_file_path, "r") as f:
            config = json.load(f)
            return config
    except FileNotFoundError:
        logger.error(f"Error: The configuration file {config_file_path} was not found.")
        return None
    except json.JSONDecodeError as e:
        logger.error(f"Error decoding JSON in {config_file_path}: {e}")
        return None
    except Exception as e:
        logger.error(
            f"An unexpected error occurred while reading config file {config_file_path}: {e}"
        )
        return None


def csv_to_df(path, file, index_col=None):
    try:
        fname = os.path.join(path, file)
        df = pd.read_csv(fname, index_col=index_col)
        return df
    except FileNotFoundError:
        logger.error(f"Error: CSV file not found at {os.path.join(path, file)}")
        return None
    except Exception as e:
        logger.error(
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
        logger.error(
            f"Error: XML file not found at {os.path.join(folder_path, filename)}"
        )
        return None
    except Exception as e:
        logger.error(
            f"An error occurred while reading XML {os.path.join(folder_path, filename)}: {e}"
        )
        return None
