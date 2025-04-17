from flask import Flask, request, jsonify, Response, stream_with_context  # type: ignore
import json
import os
import logging
import time
import threading
from queue import Queue, Empty
import atexit

# --- Configuration ---
HOST_FILE_PATH = os.path.join(
    os.path.dirname(__file__), "hosts.json"
)  # Persistent storage {ip: name}
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000
HEARTBEAT_TIMEOUT = 90  # Seconds before considering a robot disconnected
CHECK_INTERVAL = 30  # Seconds between checking for timeouts
# --- End Configuration ---

app = Flask(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - [%(threadName)s] - %(message)s",
)
logger = logging.getLogger(__name__)

# --- Shared State (Protected by Locks) ---
# robot_registry: { name: {'ip': ip, 'last_seen': timestamp} } - In-memory state
robot_registry = {}
registry_lock = threading.Lock()

# sse_clients: { clientId: Queue } - Queues for broadcasting updates
sse_clients = {}
sse_clients_lock = threading.Lock()
sse_client_id_counter = 0
# --- End Shared State ---

# --- Helper Functions ---


def _get_current_hosts_view():
    """Returns a snapshot of the current {ip: name} mapping."""
    # Assumes registry_lock is held *outside* this function if needed for atomicity
    # For read-only snapshot, often okay without lock, but safer with if called during updates
    with registry_lock:
        return {data["ip"]: name for name, data in robot_registry.items()}


def load_hosts_from_file():
    """Loads {ip: name} from file into the in-memory registry on startup."""
    global robot_registry
    if not os.path.exists(HOST_FILE_PATH):
        logger.info(f"Host file {HOST_FILE_PATH} not found, starting empty.")
        return

    try:
        with open(HOST_FILE_PATH, "r") as f:
            content = f.read()
            loaded_hosts = json.loads(content) if content else {}
    except Exception as e:
        logger.error(f"Error loading hosts file {HOST_FILE_PATH}: {e}. Starting empty.")
        loaded_hosts = {}

    # Populate in-memory registry
    current_time = time.time()
    temp_registry = {}
    for ip, name in loaded_hosts.items():
        if name and ip:  # Basic validation
            temp_registry[name] = {"ip": ip, "last_seen": current_time}
        else:
            logger.warning(
                f"Skipping invalid entry (IP: {ip}, Name: {name}) from file."
            )

    with registry_lock:
        robot_registry = temp_registry  # Atomically replace registry
    logger.info(f"Loaded {len(robot_registry)} hosts into registry from file.")


def save_hosts_to_file():
    """Saves the current {ip: name} state from registry to file."""
    hosts_to_save = _get_current_hosts_view()  # Get snapshot safely
    try:
        with open(HOST_FILE_PATH, "w") as f:
            json.dump(hosts_to_save, f, indent=4)
        logger.debug(f"Saved hosts data to {HOST_FILE_PATH}")
    except Exception as e:
        logger.error(f"Error saving hosts file {HOST_FILE_PATH}: {e}")


def broadcast_update():
    """Sends the current host list snapshot to all connected SSE clients."""
    current_hosts_view = _get_current_hosts_view()  # Get snapshot
    message = f"data: {json.dumps(current_hosts_view)}\n\n"
    logger.info(f"Broadcasting update to {len(sse_clients)} SSE clients.")
    logger.debug(f"Update message payload: {current_hosts_view}")

    with sse_clients_lock:
        client_ids = list(sse_clients.keys())  # Copy keys for safe iteration

    for client_id in client_ids:
        with sse_clients_lock:
            q = sse_clients.get(client_id)
        if q:
            try:
                q.put_nowait(message)
            except Exception as e:  # Should be rare with Queue
                logger.warning(
                    f"Could not put message in queue for client {client_id}: {e}"
                )
        # else: Queue already removed by disconnection logic


def _register_or_update_robot(name, ip):
    """Handles adding or updating a robot in the registry. Returns if broadcast needed."""
    needs_broadcast = False
    current_time = time.time()
    with registry_lock:
        # Check for IP conflict FIRST
        for existing_name, existing_data in robot_registry.items():
            if existing_data["ip"] == ip and existing_name != name:
                logger.warning(
                    f"IP conflict: {ip} already used by '{existing_name}'. Cannot register '{name}'."
                )
                return False, "ip_conflict", existing_name  # Indicate conflict

        if name in robot_registry:
            # Update existing entry
            if robot_registry[name]["ip"] != ip:
                logger.info(
                    f"Updating IP for '{name}' from {robot_registry[name]['ip']} to {ip}"
                )
                robot_registry[name]["ip"] = ip
                needs_broadcast = True  # IP changed, notify others
            robot_registry[name]["last_seen"] = current_time  # Always update timestamp
            logger.debug(f"Refreshed registration for '{name}'")
        else:
            # Add new entry
            logger.info(f"Registering new robot: '{name}' with IP {ip}")
            robot_registry[name] = {"ip": ip, "last_seen": current_time}
            needs_broadcast = True  # New robot joined, notify others

    # Save and broadcast happen *after* releasing lock
    if needs_broadcast:
        save_hosts_to_file()
        broadcast_update()
    else:
        # Still save if only last_seen was updated (e.g., re-register without IP change)
        save_hosts_to_file()

    return True, "ok", name  # Indicate success


def _unregister_robot(name):
    """Internal function to remove a robot (handles state and broadcast)."""
    removed = False
    ip = None
    with registry_lock:
        if name in robot_registry:
            ip = robot_registry.pop(name)["ip"]  # Remove and get IP
            removed = True
            logger.info(f"Unregistered '{name}' (IP: {ip}) from registry.")

    if removed:
        save_hosts_to_file()
        broadcast_update()  # Notify clients of the change
    return removed, ip


# --- Flask Endpoints ---


@app.route("/register", methods=["POST"])
def register_robot_endpoint():
    data = request.get_json()
    if not data or "name" not in data or "ip" not in data:
        return jsonify({"error": "Request must be JSON with 'name' and 'ip'"}), 400

    name = data["name"]
    ip = data["ip"]
    logger.info(f"Received registration request: Name={name}, IP={ip}")

    success, reason, detail = _register_or_update_robot(name, ip)

    if success:
        with registry_lock:  # Get neighbors *after* potential update
            neighbors = {d["ip"]: n for n, d in robot_registry.items() if n != name}
        return (
            jsonify({"message": "Registration successful", "neighbors": neighbors}),
            200,
        )
    elif reason == "ip_conflict":
        return (
            jsonify({"error": f"IP address {ip} already registered to '{detail}'"}),
            409,
        )
    else:
        # Should not happen with current logic, but for completeness
        return jsonify({"error": "Registration failed for an unknown reason"}), 500


@app.route("/hosts", methods=["GET"])
def get_hosts_endpoint():
    logger.debug("Received request for /hosts")
    return jsonify(_get_current_hosts_view()), 200


@app.route("/unregister/<name>", methods=["DELETE"])
def unregister_robot_endpoint(name):
    logger.info(f"Received explicit unregister request for: {name}")
    removed, _ = _unregister_robot(name)
    if removed:
        return jsonify({"message": f"'{name}' unregistered successfully"}), 200
    else:
        logger.warning(f"Attempted to unregister '{name}', but it was not found.")
        return jsonify({"error": f"Host '{name}' not found"}), 404


@app.route("/heartbeat", methods=["POST"])
def heartbeat_endpoint():
    data = request.get_json()
    if not data or "name" not in data:
        return jsonify({"error": "Request must be JSON with 'name'"}), 400

    name = data["name"]
    updated = False
    with registry_lock:
        if name in robot_registry:
            robot_registry[name]["last_seen"] = time.time()
            updated = True
            logger.debug(f"Heartbeat received for: {name}")
        else:
            logger.warning(f"Heartbeat received for unknown robot: {name}")

    if updated:
        # No save/broadcast on heartbeat to avoid excessive load
        return jsonify({"message": "Heartbeat acknowledged"}), 200
    else:
        # Consider if a heartbeat from an unknown robot should trigger re-registration prompt?
        # For now, return 404 as it's not *currently* registered.
        return jsonify({"error": f"Robot '{name}' not currently registered"}), 404


@app.route("/events")
def sse_events_endpoint():
    global sse_client_id_counter

    # Assign unique ID and create a queue for this client
    with sse_clients_lock:
        client_id = sse_client_id_counter
        sse_client_id_counter += 1
        client_q = Queue()
        sse_clients[client_id] = client_q
        logger.info(
            f"SSE Client {client_id} connected. Total clients: {len(sse_clients)}"
        )

    # Send initial state immediately using a specific event type
    initial_hosts = _get_current_hosts_view()
    initial_message = f"event: initial_state\ndata: {json.dumps(initial_hosts)}\n\n"
    try:
        client_q.put_nowait(initial_message)
    except Exception as e:
        logger.warning(
            f"Could not put initial message in queue for client {client_id}: {e}"
        )

    @stream_with_context
    def generate():
        try:
            while True:
                try:
                    # Wait for messages. Timeout helps keep connection somewhat active
                    # and allows checking loop condition more often.
                    message = client_q.get(timeout=60)
                    yield message
                except Empty:
                    # Send a comment to keep the connection alive if no updates
                    yield ": keepalive\n\n"
        except GeneratorExit:
            # Client disconnected
            logger.info(f"SSE Client {client_id} disconnected.")
        finally:
            # Clean up the client's queue
            with sse_clients_lock:
                if client_id in sse_clients:
                    del sse_clients[client_id]
                    logger.info(
                        f"Removed queue for SSE Client {client_id}. Remaining clients: {len(sse_clients)}"
                    )

    return Response(generate(), mimetype="text/event-stream")


# --- Background Task for Timeout Check ---
class HeartbeatChecker(threading.Thread):
    def __init__(self, check_interval=CHECK_INTERVAL, timeout=HEARTBEAT_TIMEOUT):
        super().__init__(daemon=True, name="HeartbeatChecker")
        self.check_interval = check_interval
        self.timeout = timeout
        self._stop_event = threading.Event()
        logger.info(
            f"HeartbeatChecker initialized (Interval: {check_interval}s, Timeout: {timeout}s)"
        )

    def stop(self):
        self._stop_event.set()

    def run(self):
        logger.info("HeartbeatChecker thread started.")
        while not self._stop_event.wait(
            self.check_interval
        ):  # Efficiently wait or break if stopped
            try:
                now = time.time()
                timed_out_names = []

                # Check for timeouts - work on a snapshot of names
                with registry_lock:
                    names_to_check = list(robot_registry.keys())

                for name in names_to_check:
                    with (
                        registry_lock
                    ):  # Re-acquire lock to check specific robot's time
                        if name in robot_registry:  # Check if still exists
                            last_seen = robot_registry[name]["last_seen"]
                            if now - last_seen > self.timeout:
                                logger.warning(
                                    f"Robot '{name}' timed out (last seen {now - last_seen:.1f}s ago)."
                                )
                                timed_out_names.append(name)
                        # else: Robot was unregistered between getting the list and checking it. Ignore.

                # Unregister timed-out robots outside the check loop
                if timed_out_names:
                    logger.info(
                        f"Unregistering {len(timed_out_names)} timed-out robots: {timed_out_names}"
                    )
                    for name in timed_out_names:
                        _unregister_robot(name)  # Handles locking, saving, broadcasting

            except Exception as e:
                logger.error(f"Error in HeartbeatChecker loop: {e}", exc_info=True)

        logger.info("HeartbeatChecker thread stopped.")


# --- Main Execution ---
if __name__ == "__main__":
    load_hosts_from_file()  # Load initial state

    checker = HeartbeatChecker()
    checker.start()

    # Graceful shutdown handler
    def shutdown_server():
        logger.info("Server shutting down...")
        checker.stop()
        checker.join(timeout=5)
        logger.info("Server shutdown complete.")

    atexit.register(shutdown_server)

    logger.info(f"Starting Flask server on {SERVER_HOST}:{SERVER_PORT}")
    # Use Waitress or Gunicorn for production
    # app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True)
    # Using waitress for demonstration (pip install waitress)
    from waitress import serve  # type: ignore

    serve(app, host=SERVER_HOST, port=SERVER_PORT, threads=8)  # Example with waitress

# --- END OF FILE fleetUpdateMngr.py ---
