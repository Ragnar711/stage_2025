import socket
import json
import os
import pandas as pd  # type: ignore
import requests
import logging
import time
import threading
import atexit
from sseclient import SSEClient  # type: ignore

# --- Client Configuration ---
# Use environment variables with defaults
HOSTS_SERVER_URL = os.environ.get(
    "HOSTS_SERVER_URL", "http://localhost:5000"
)  # Default to localhost for easier dev
ROBOT_NAME = os.environ.get(
    "ROBOT_NAME", f"robot_{int(time.time()) % 10000}"
)  # Default to semi-unique name
REQUEST_TIMEOUT = 10  # Seconds for normal HTTP requests
HEARTBEAT_INTERVAL = 30  # Seconds between sending heartbeats
SSE_RECONNECT_DELAY = 5  # Seconds before attempting SSE reconnect on error
# --- End Client Configuration ---

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - [%(threadName)s] - %(message)s",
)
# Use a logger specific to this client instance
logger = logging.getLogger(f"Client.{ROBOT_NAME}")

# --- Global variable for the singleton HostMonitor instance ---
_host_monitor_instance = None
_monitor_lock = threading.Lock()
# ---


def get_local_ip_address():
    """Tries to determine the local IP address connected to the internet."""
    # This is a common approach but might not be perfect in complex network setups.
    s = None
    ip_address = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1)
        # Doesn't actually send data, just tries to find route
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
    except OSError as e:
        logger.warning(
            f"Could not determine local IP address via socket: {e}. Trying hostname."
        )
        try:
            # Fallback using hostname (might return loopback)
            ip_address = socket.gethostbyname(socket.gethostname())
        except socket.gaierror:
            logger.error("Could not determine IP via socket or hostname.")
            ip_address = "127.0.0.1"  # Last resort fallback
    finally:
        if s:
            s.close()
    logger.info(f"Determined local IP as: {ip_address}")
    return ip_address


# =============================================================================
# == Host Monitor Class and Functions ==
# =============================================================================


class HostMonitor:
    """
    Manages connection, registration, heartbeats, and neighbor updates
    by interacting with the fleetUpdateMngr server.
    """

    def __init__(self, server_url, robot_name, heartbeat_interval, req_timeout):
        self.server_url = server_url.rstrip("/")  # Ensure no trailing slash
        self.robot_name = robot_name
        self.heartbeat_interval = heartbeat_interval
        self.req_timeout = req_timeout
        self.robot_ip = None  # Determined during registration

        self._neighbors = {}  # Stores current {ip: name} of neighbors
        self._neighbors_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sse_thread = None
        self._heartbeat_thread = None
        self.session = requests.Session()  # Use a session for connection pooling
        self.is_registered = False

    def _update_neighbors(self, all_hosts_data):
        """Safely updates the internal neighbor list from server data."""
        if not isinstance(all_hosts_data, dict):
            logger.warning(f"Received non-dict data for host update: {all_hosts_data}")
            return

        with self._neighbors_lock:
            self._neighbors.clear()
            for ip, name in all_hosts_data.items():
                # Exclude self from neighbors list
                if name != self.robot_name or ip != self.robot_ip:
                    self._neighbors[ip] = name
            logger.info(f"Neighbors updated: {list(self._neighbors.values())}")
            # Potential enhancement: Add a callback hook here if external code
            # needs immediate notification of neighbor changes.
            # E.g., if self.on_update_callback: self.on_update_callback(self.get_neighbors_with_ips())

    def register(self):
        """Registers this robot with the server. Returns True on success."""
        if self.is_registered:
            logger.info("Already registered.")
            return True

        self.robot_ip = get_local_ip_address()
        if not self.robot_ip:
            logger.critical("Failed to determine local IP. Cannot register.")
            return False

        register_url = f"{self.server_url}/register"
        payload = {"name": self.robot_name, "ip": self.robot_ip}
        try:
            logger.info(f"Attempting registration: {payload} to {register_url}")
            response = self.session.post(
                register_url, json=payload, timeout=self.req_timeout
            )
            response.raise_for_status()  # Raises HTTPError for 4xx/5xx
            response_data = response.json()
            logger.info(
                f"Registration successful: {response_data.get('message', '(no message)')}"
            )
            # Initial neighbors list from registration response
            self._update_neighbors(response_data.get("neighbors", {}))
            self.is_registered = True
            return True
        except requests.exceptions.HTTPError as e:
            logger.error(
                f"Registration failed: {e.response.status_code} from {register_url}"
            )
            try:
                logger.error(f"Server response: {e.response.json()}")
            except json.JSONDecodeError:
                logger.error(f"Server response (non-JSON): {e.response.text}")
            self.is_registered = False
            return False
        except requests.exceptions.RequestException as e:
            logger.error(
                f"Failed to connect to server for registration at {register_url}: {e}"
            )
            self.is_registered = False
            return False
        except Exception as e:
            logger.error(
                f"An unexpected error occurred during registration: {e}", exc_info=True
            )
            self.is_registered = False
            return False

    def _listen_for_updates(self):
        """Background thread function to listen for SSE updates."""
        sse_url = f"{self.server_url}/events"
        logger.info(f"SSE listener connecting to {sse_url}")
        while not self._stop_event.is_set():
            stream_client = None
            try:
                # Connect with stream=True
                response = self.session.get(
                    sse_url, stream=True, timeout=self.req_timeout + 5
                )
                response.raise_for_status()
                stream_client = SSEClient(response)
                logger.info(f"SSE connection established to {sse_url}")

                for event in stream_client.events():  # Blocks until event/error
                    if self._stop_event.is_set():
                        break

                    logger.debug(f"SSE received: type={event.event}, data={event.data}")
                    if (
                        event.event == "message" or event.event == "initial_state"
                    ):  # Handle default 'message' and our custom initial
                        if event.data:
                            try:
                                all_hosts = json.loads(event.data)
                                self._update_neighbors(all_hosts)
                            except json.JSONDecodeError:
                                logger.error(
                                    f"Failed to decode JSON from SSE event: {event.data}"
                                )
                            except Exception as e:
                                logger.error(
                                    f"Error processing SSE data: {e}", exc_info=True
                                )
                    # Can add handlers for other event.event types if needed

            except requests.exceptions.ConnectionError as e:
                logger.warning(
                    f"SSE connection error to {sse_url}: {e}. Retrying in {SSE_RECONNECT_DELAY}s..."
                )
            except requests.exceptions.HTTPError as e:
                logger.warning(
                    f"SSE HTTP error from {sse_url}: {e.response.status_code}. Retrying in {SSE_RECONNECT_DELAY}s..."
                )
                if e.response:
                    logger.error(f"Server response: {e.response.text}")
            except requests.exceptions.RequestException as e:
                logger.warning(
                    f"SSE request error for {sse_url}: {e}. Retrying in {SSE_RECONNECT_DELAY}s..."
                )
            except Exception as e:
                # Catch potential errors within SSEClient loop or unexpected issues
                logger.error(f"Unexpected error in SSE listener: {e}", exc_info=True)
                logger.info(f"Retrying SSE connection in {SSE_RECONNECT_DELAY}s...")
            finally:
                if stream_client:  # Ensure underlying connection gets closed
                    stream_client.close()

            # Wait before retrying connection or if loop exited unexpectedly
            if not self._stop_event.is_set():
                self._stop_event.wait(SSE_RECONNECT_DELAY)

        logger.info("SSE listener thread stopped.")

    def _send_heartbeats(self):
        """Background thread function to send periodic heartbeats."""
        heartbeat_url = f"{self.server_url}/heartbeat"
        payload = {"name": self.robot_name}
        logger.info(
            f"Heartbeat thread starting. Sending to {heartbeat_url} every {self.heartbeat_interval}s"
        )

        while not self._stop_event.wait(self.heartbeat_interval):
            if not self.is_registered:
                logger.warning("Not registered, skipping heartbeat.")
                continue  # Try again next interval, maybe registration succeeded by then

            try:
                logger.debug(f"Sending heartbeat: {payload}")
                response = self.session.post(
                    heartbeat_url, json=payload, timeout=self.req_timeout
                )
                # 404 is possible if server lost registration; attempt re-register
                if response.status_code == 404:
                    logger.warning(
                        f"Heartbeat rejected (404), server may have lost registration for '{self.robot_name}'. Attempting re-registration."
                    )
                    self.is_registered = False  # Mark as not registered
                    if not self.register():  # Try immediate re-registration
                        logger.error(
                            "Re-registration attempt failed after heartbeat rejection."
                        )
                    # Whether re-registration succeeds or fails, continue the loop
                else:
                    response.raise_for_status()  # Check for other 4xx/5xx errors
                    logger.debug("Heartbeat acknowledged.")
            except requests.exceptions.RequestException as e:
                logger.warning(f"Failed to send heartbeat to {heartbeat_url}: {e}")
                # Consider more robust error handling? Maybe trigger re-registration after N failures?
            except Exception as e:
                logger.error(f"Unexpected error sending heartbeat: {e}", exc_info=True)

        logger.info("Heartbeat thread stopped.")

    def start(self):
        """Registers the robot and starts background threads for SSE and heartbeats."""
        if self._sse_thread or self._heartbeat_thread:
            logger.warning(
                "Monitor threads seem to be already started or not cleaned up properly."
            )
            # Optionally add logic to stop existing threads first if restarting is intended
            # self.stop() # Uncomment if restart should force stop first

        logger.info("Starting Host Monitor...")
        if not self.register():  # Attempt initial registration
            logger.error(
                "Initial registration failed. Monitor will not start background tasks."
            )
            # Could implement retries here if desired
            return False  # Indicate failure to start

        # Start threads only after successful registration
        self._stop_event.clear()
        self._sse_thread = threading.Thread(
            target=self._listen_for_updates,
            daemon=True,
            name=f"{self.robot_name}-SSEListener",
        )
        self._heartbeat_thread = threading.Thread(
            target=self._send_heartbeats,
            daemon=True,
            name=f"{self.robot_name}-HeartbeatSender",
        )
        self._sse_thread.start()
        self._heartbeat_thread.start()
        logger.info("Host Monitor background threads started.")
        return True  # Indicate success

    def stop(self):
        """Signals threads to stop, attempts unregistration, and cleans up."""
        logger.info("Stopping Host Monitor...")
        self._stop_event.set()  # Signal threads

        # Attempt explicit unregistration if we were registered
        if (
            self.is_registered and self.robot_ip
        ):  # Only unregister if we successfully registered
            unregister_url = f"{self.server_url}/unregister/{self.robot_name}"
            try:
                logger.info(f"Attempting explicit unregistration from {unregister_url}")
                response = self.session.delete(unregister_url, timeout=self.req_timeout)
                if response.ok:
                    logger.info(
                        f"Unregistration request successful: {response.status_code}"
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
        self._sse_thread = None  # Clear thread variable

        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            logger.debug("Waiting for Heartbeat thread to join...")
            self._heartbeat_thread.join(timeout=1)
            if self._heartbeat_thread.is_alive():
                logger.warning("Heartbeat thread did not join cleanly.")
        self._heartbeat_thread = None  # Clear thread variable

        self.session.close()
        self.is_registered = False
        logger.info("Host Monitor stopped.")

    def get_neighbors(self):
        """Returns a list of current neighbor names."""
        with self._neighbors_lock:
            return list(self._neighbors.values())

    def get_neighbors_with_ips(self):
        """Returns a dictionary of {ip: name} for current neighbors."""
        with self._neighbors_lock:
            return self._neighbors.copy()


# --- Singleton Management Functions ---


def start_host_monitor():
    """Initializes and starts the singleton HostMonitor instance."""
    global _host_monitor_instance
    with _monitor_lock:
        if _host_monitor_instance is None:
            logger.info("Creating and starting singleton Host Monitor instance...")
            monitor = HostMonitor(
                server_url=HOSTS_SERVER_URL,
                robot_name=ROBOT_NAME,
                heartbeat_interval=HEARTBEAT_INTERVAL,
                req_timeout=REQUEST_TIMEOUT,
            )
            if monitor.start():  # Start returns True on success
                _host_monitor_instance = monitor
                # Register the stop function to be called at Python exit
                atexit.register(stop_host_monitor)
                logger.info("Host monitor started successfully.")
            else:
                logger.error(
                    "Host monitor failed to start (likely registration issue)."
                )
                # _host_monitor_instance remains None
        else:
            logger.debug("Host Monitor instance already exists.")
        return _host_monitor_instance


def stop_host_monitor():
    """Stops the singleton HostMonitor instance if it exists."""
    global _host_monitor_instance
    with _monitor_lock:
        if _host_monitor_instance is not None:
            logger.info(
                "Stopping singleton Host Monitor instance via stop_host_monitor()..."
            )
            _host_monitor_instance.stop()
            _host_monitor_instance = None
        # else: No instance to stop


def get_robot_name():
    """Gets the configured robot name."""
    return ROBOT_NAME


def get_current_neighbors():
    """
    Gets the latest list of neighbor names. Starts monitor if needed.
    Returns empty list if monitor cannot be started or has no neighbors.
    """
    monitor = _host_monitor_instance or start_host_monitor()
    if monitor:
        return monitor.get_neighbors()
    else:
        logger.warning("Host monitor not available. Returning empty neighbors list.")
        return []


def get_current_neighbors_with_ips():
    """
    Gets the latest {ip: name} dictionary for neighbors. Starts monitor if needed.
    Returns empty dict if monitor cannot be started or has no neighbors.
    """
    monitor = _host_monitor_instance or start_host_monitor()
    if monitor:
        return monitor.get_neighbors_with_ips()
    else:
        logger.warning("Host monitor not available. Returning empty neighbors dict.")
        return {}


# =============================================================================
# == Unrelated File Utility Functions ==
# =============================================================================
# Note: These functions are not directly related to the Host Monitor logic
# and could ideally live in a separate utility module (e.g., file_utils.py).
# They are kept here as requested by the prompt to simplify *this* file.
# The 'pandas' import belongs with these utilities.


def read_config_file(config_file_path):
    """Reads a JSON configuration file."""
    logger.debug(f"Attempting to read JSON config: {config_file_path}")
    try:
        with open(config_file_path, "r") as f:
            config = json.load(f)
            return config
    except FileNotFoundError:
        logger.error(f"Config file not found: {config_file_path}")
        return None
    except json.JSONDecodeError as e:
        logger.error(f"Error decoding JSON in {config_file_path}: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error reading config {config_file_path}: {e}")
        return None


def csv_to_df(path, file, index_col=None):
    """Reads a CSV file into a pandas DataFrame."""
    fname = os.path.join(path, file)
    logger.debug(f"Attempting to read CSV: {fname}")
    try:
        df = pd.read_csv(fname, index_col=index_col)
        return df
    except FileNotFoundError:
        logger.error(f"CSV file not found: {fname}")
        return None
    except pd.errors.EmptyDataError:
        logger.warning(f"CSV file is empty: {fname}")
        return pd.DataFrame()  # Return empty DataFrame for empty file
    except Exception as e:
        logger.error(f"Error reading CSV {fname}: {e}")
        return None


def read_xml_file(folder_path, filename):
    """Reads the content of an XML file."""
    filepath = os.path.join(folder_path, filename)
    logger.debug(f"Attempting to read XML file: {filepath}")
    try:
        with open(filepath, "r") as f:
            xml_content = f.read()
        return xml_content
    except FileNotFoundError:
        logger.error(f"XML file not found: {filepath}")
        return None
    except Exception as e:
        logger.error(f"Error reading XML {filepath}: {e}")
        return None


# --- END OF FILE manageFiles.py ---
