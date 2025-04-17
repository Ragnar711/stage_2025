# hosts_server.py (Enhanced)
from flask import Flask, request, jsonify, Response, stream_with_context
import json
import os
import logging
import time
import threading
from queue import Queue, Empty
from copy import deepcopy

# --- Configuration ---
HOST_FILE_PATH = 'hosts.json' # Persistent storage {ip: name}
SERVER_HOST = '0.0.0.0'
SERVER_PORT = 5000
HEARTBEAT_TIMEOUT = 90  # Seconds before considering a robot disconnected
CHECK_INTERVAL = 30     # Seconds between checking for timeouts
# --- End Configuration ---

app = Flask(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - [%(threadName)s] - %(message)s')

# --- Shared State (Protected by Locks) ---
# robot_registry: { name: {'ip': ip, 'last_seen': timestamp} } - In-memory state
robot_registry = {}
registry_lock = threading.Lock()

# sse_clients: { clientId: Queue } - Queues for broadcasting updates
sse_clients = {}
sse_clients_lock = threading.Lock()
sse_client_id_counter = 0
# --- End Shared State ---

def load_hosts_from_file():
    """Loads {ip: name} from file into the in-memory registry on startup."""
    global robot_registry
    if not os.path.exists(HOST_FILE_PATH):
        logging.info(f"Host file {HOST_FILE_PATH} not found, starting empty.")
        return {}

    with registry_lock: # Lock while modifying registry
        loaded_hosts = {}
        try:
            with open(HOST_FILE_PATH, 'r') as f:
                content = f.read()
                if content:
                    loaded_hosts = json.loads(content)
                else: # Handle empty file
                     loaded_hosts = {}
        except json.JSONDecodeError:
            logging.error(f"Error decoding JSON from {HOST_FILE_PATH}. Starting empty.")
            loaded_hosts = {}
        except Exception as e:
            logging.error(f"Error loading hosts file {HOST_FILE_PATH}: {e}")
            loaded_hosts = {}

        # Populate in-memory registry, setting initial last_seen
        current_time = time.time()
        robot_registry.clear() # Clear previous in-memory state
        for ip, name in loaded_hosts.items():
            # Basic validation: ensure name isn't empty
            if name:
                 robot_registry[name] = {'ip': ip, 'last_seen': current_time}
            else:
                logging.warning(f"Skipping entry with empty name for IP {ip} from file.")
        logging.info(f"Loaded {len(robot_registry)} hosts into registry from file.")

def save_hosts_to_file():
    """Saves the current {ip: name} state from registry to file."""
    # No lock needed here if we read from registry *with* the lock
    with registry_lock:
        hosts_to_save = {data['ip']: name for name, data in robot_registry.items()}

    # File writing itself doesn't need the registry lock, but could have its own lock if needed
    try:
        with open(HOST_FILE_PATH, 'w') as f:
            json.dump(hosts_to_save, f, indent=4)
        logging.debug(f"Saved hosts data to {HOST_FILE_PATH}: {hosts_to_save}")
    except Exception as e:
        logging.error(f"Error saving hosts file {HOST_FILE_PATH}: {e}")

def broadcast_update():
    """Sends the current host list to all connected SSE clients."""
    with registry_lock:
        # Create a snapshot of the current state to broadcast
        current_hosts_view = {data['ip']: name for name, data in robot_registry.items()}

    message = f"data: {json.dumps(current_hosts_view)}\n\n"
    logging.info(f"Broadcasting update to {len(sse_clients)} SSE clients.")
    logging.debug(f"Update message: {message.strip()}")

    # Iterate over a copy of client IDs in case a client disconnects during broadcast
    with sse_clients_lock:
        client_ids = list(sse_clients.keys())

    for client_id in client_ids:
        with sse_clients_lock:
            q = sse_clients.get(client_id) # Get queue safely
        if q:
            try:
                q.put_nowait(message) # Put message in the client's queue
            except Exception as e:
                # Should not happen with Queue unless full, but good practice
                logging.warning(f"Could not put message in queue for client {client_id}: {e}")
        else:
             logging.warning(f"Client {client_id} queue not found during broadcast.")


def _unregister_robot_internal(name):
    """Internal function to remove a robot (handles state and broadcast)."""
    removed = False
    ip = "unknown"
    with registry_lock:
        if name in robot_registry:
            ip = robot_registry[name]['ip']
            del robot_registry[name]
            removed = True
            logging.info(f"Unregistered '{name}' (IP: {ip}) from registry.")

    if removed:
        save_hosts_to_file()
        broadcast_update() # Notify clients of the change
    return removed, ip

# --- Flask Endpoints ---

@app.route('/register', methods=['POST'])
def register_robot():
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    name = data.get('name')
    ip = data.get('ip')

    if not name or not ip:
        return jsonify({"error": "Missing 'name' or 'ip' in JSON payload"}), 400

    logging.info(f"Received registration request: Name={name}, IP={ip}")
    needs_broadcast = False
    current_time = time.time()

    with registry_lock:
        # Check for IP conflict
        for existing_name, existing_data in robot_registry.items():
            if existing_data['ip'] == ip and existing_name != name:
                logging.warning(f"IP address {ip} is already registered to '{existing_name}'. Cannot register '{name}' with this IP.")
                return jsonify({"error": f"IP address {ip} already registered to '{existing_name}'"}), 409

        # Add or update the robot
        if name in robot_registry:
             # Update existing entry
             if robot_registry[name]['ip'] != ip:
                  logging.info(f"Updating IP for '{name}' from {robot_registry[name]['ip']} to {ip}")
                  needs_broadcast = True # IP changed, notify others
             robot_registry[name]['ip'] = ip
             robot_registry[name]['last_seen'] = current_time
        else:
             # Add new entry
             logging.info(f"Registering new robot: '{name}' with IP {ip}")
             robot_registry[name] = {'ip': ip, 'last_seen': current_time}
             needs_broadcast = True # New robot joined, notify others

        # Create response data *while holding the lock*
        neighbors = {data['ip']: n for n, data in robot_registry.items() if n != name}

    # Save and broadcast *after* releasing the lock
    if needs_broadcast:
        save_hosts_to_file()
        broadcast_update()
    else:
        # Still save if only last_seen was updated (e.g., re-register)
        save_hosts_to_file()


    return jsonify({"message": "Registration successful", "neighbors": neighbors}), 200

@app.route('/hosts', methods=['GET'])
def get_hosts():
    logging.debug("Received request for /hosts")
    with registry_lock:
        # Return the simple {ip: name} view
        current_hosts_view = {data['ip']: name for name, data in robot_registry.items()}
    return jsonify(current_hosts_view), 200

# Explicit unregister
@app.route('/unregister/<name>', methods=['DELETE'])
def unregister_robot_endpoint(name):
    logging.info(f"Received explicit unregister request for: {name}")
    removed, ip = _unregister_robot_internal(name)
    if removed:
        return jsonify({"message": f"'{name}' unregistered successfully"}), 200
    else:
        logging.warning(f"Attempted to unregister '{name}', but it was not found.")
        return jsonify({"error": f"Host '{name}' not found"}), 404

# Heartbeat endpoint
@app.route('/heartbeat', methods=['POST'])
def heartbeat():
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    name = data.get('name')

    if not name:
        return jsonify({"error": "Missing 'name' in JSON payload"}), 400

    updated = False
    with registry_lock:
        if name in robot_registry:
            robot_registry[name]['last_seen'] = time.time()
            updated = True
            logging.debug(f"Heartbeat received and updated for: {name}")
        else:
            logging.warning(f"Heartbeat received for unknown robot: {name}")

    if updated:
        # Optional: Save file on heartbeat? Maybe too frequent.
        # save_hosts_to_file()
        return jsonify({"message": "Heartbeat acknowledged"}), 200
    else:
        # Treat heartbeat from unknown robot as potential re-registration needed?
        # Or just ignore it. Let's ignore for now.
         return jsonify({"error": f"Robot '{name}' not currently registered"}), 404


# Server-Sent Events endpoint
@app.route('/events')
def sse_events():
    global sse_client_id_counter

    # Assign unique ID and create a queue for this client
    with sse_clients_lock:
        client_id = sse_client_id_counter
        sse_client_id_counter += 1
        client_q = Queue()
        sse_clients[client_id] = client_q
        logging.info(f"SSE Client {client_id} connected. Total clients: {len(sse_clients)}")

    # Send initial state immediately
    with registry_lock:
        initial_hosts = {data['ip']: name for name, data in robot_registry.items()}
    initial_message = f"event: initial_state\ndata: {json.dumps(initial_hosts)}\n\n"
    try:
         client_q.put_nowait(initial_message)
    except Exception as e:
         logging.warning(f"Could not put initial message in queue for client {client_id}: {e}")


    def generate():
        try:
            while True:
                # Wait for a message, timeout helps detect disconnects sooner potentially
                try:
                    message = client_q.get(timeout=30) # Wait up to 30s for a message
                    yield message
                except Empty:
                    # No message, send a comment to keep connection alive? Optional.
                    # yield ": keepalive\n\n"
                    pass # Just continue loop to check again
        except GeneratorExit:
            # Client disconnected
            logging.info(f"SSE Client {client_id} disconnected.")
        finally:
            # Clean up the client's queue
            with sse_clients_lock:
                if client_id in sse_clients:
                    del sse_clients[client_id]
                    logging.info(f"Removed queue for SSE Client {client_id}. Remaining clients: {len(sse_clients)}")

    # Use stream_with_context for proper handling within Flask request context
    return Response(stream_with_context(generate()), mimetype='text/event-stream')

# --- Background Task for Timeout Check ---
class HeartbeatChecker(threading.Thread):
    def __init__(self, check_interval=CHECK_INTERVAL, timeout=HEARTBEAT_TIMEOUT):
        super().__init__(daemon=True, name="HeartbeatChecker") # Daemon thread exits when main thread exits
        self.check_interval = check_interval
        self.timeout = timeout
        self._stop_event = threading.Event()
        logging.info(f"HeartbeatChecker initialized (Interval: {check_interval}s, Timeout: {timeout}s)")

    def stop(self):
        self._stop_event.set()

    def run(self):
        logging.info("HeartbeatChecker thread started.")
        while not self._stop_event.is_set():
            try:
                now = time.time()
                timed_out_robots = []

                # Check for timeouts - work on a copy of names to avoid modification issues
                with registry_lock:
                    robot_names = list(robot_registry.keys())

                for name in robot_names:
                    with registry_lock: # Re-acquire lock to check specific robot
                        if name in robot_registry: # Check if still exists (might have been unregistered)
                            last_seen = robot_registry[name]['last_seen']
                            if now - last_seen > self.timeout:
                                logging.warning(f"Robot '{name}' timed out (last seen {now - last_seen:.1f}s ago).")
                                timed_out_robots.append(name)
                        else:
                            continue # Robot already gone

                # Unregister timed-out robots (outside the main check loop)
                if timed_out_robots:
                    logging.info(f"Found {len(timed_out_robots)} timed-out robots: {timed_out_robots}")
                    for name in timed_out_robots:
                        # This function handles locking, saving, and broadcasting
                        _unregister_robot_internal(name)

            except Exception as e:
                logging.error(f"Error in HeartbeatChecker loop: {e}", exc_info=True)

            # Wait for the next interval or until stopped
            self._stop_event.wait(self.check_interval)
        logging.info("HeartbeatChecker thread stopped.")

# --- Main Execution ---
if __name__ == '__main__':
    load_hosts_from_file() # Load initial state

    # Start the background checker
    checker = HeartbeatChecker()
    checker.start()

    logging.info(f"Starting Flask server on {SERVER_HOST}:{SERVER_PORT}")
    # Use a production server like Gunicorn or Waitress for real deployments
    # For development:
    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=False, threaded=True) # threaded=True is important for background tasks/SSE

    # Cleanup on exit (though daemon thread might make this less critical)
    # Note: This won't run if killed forcefully (SIGKILL)
    logging.info("Server shutting down...")
    checker.stop()
    checker.join(timeout=5) # Wait briefly for checker thread to finish
    logging.info("Server shutdown complete.")