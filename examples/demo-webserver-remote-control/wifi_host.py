"""
ESP32-based WiFi and Web Server Control for Arduino-Alvik Robot

This script sets up an ESP32 as a WiFi access point and runs both a DNS server 
(for captive portal functionality) and a web server (to receive remote control commands). 

The web server listens for requests that modify the robot's movement, which 
is controlled via the Arduino-Alvik library.

Features:
- Configures ESP32 as a SoftAP (WiFi hotspot)
- Runs a simple DNS server to handle captive portal requests
- Runs an HTTP web server to process user commands
- Provides a basic control interface for moving an Arduino-Alvik robot
- Uses threading for concurrent execution of DNS and Web server

Author: Orso Eric
Company: AROL Closure Systems
Date: 2025-05-06
License: MIT
"""

import network
import socket
import _thread
from time import sleep_ms

from arduino_alvik import ArduinoAlvik

#name of the wifi network
gs_ssid = "Alvik-robot-Arol"


def update_speed(forward, right):
    _speed_lock.acquire()
    try:
        _speed_data["forward"] = forward
        _speed_data["right"] = right
    finally:
        _speed_lock.release()

def get_speed():
    _speed_lock.acquire()
    try:
        forward = _speed_data["forward"]
        right = _speed_data["right"]
    finally:
        _speed_lock.release()
    return forward, right

def setup_access_point():
    # Configure ESP32 as a WiFi SoftAP.
    cl_access_point = network.WLAN(network.AP_IF)
    cl_access_point.active(True)
    cl_access_point.config(essid=gs_ssid, password="12345678")
    print("Access Point started with config:", cl_access_point.ifconfig())
    return cl_access_point

def build_dns_response(data):
    transaction_id = data[0:2]
    flags = b'\x81\x80'
    qdcount = data[4:6]
    ancount = b'\x00\x01'  # One answer record.
    nscount = b'\x00\x00'
    arcount = b'\x00\x00'
    header = transaction_id + flags + qdcount + ancount + nscount + arcount

    query = data[12:]
    answer = b'\xc0\x0c'     # Pointer back to the query name.
    answer += b'\x00\x01'    # Type A record.
    answer += b'\x00\x01'    # Class IN.
    answer += b'\x00\x00\x00\x3c'  # TTL of 60 seconds.
    answer += b'\x00\x04'    # IPv4 address length.
    ip = gcl_access_point.ifconfig()[0]
    ip_bytes = bytes([int(x) for x in ip.split('.')])
    answer += ip_bytes

    return header + query + answer

def dns_server():
    global gb_app_running
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", 53))
        sock.settimeout(1)
        print("DNS server listening on UDP port 53")
        while gb_app_running:
            try:
                data, addr = sock.recvfrom(512)
                print("DNS request from:", addr)
                response = build_dns_response(data)
                sock.sendto(response, addr)
            except OSError:
                # Timeout: re-check gb_app_running.
                pass
        sock.close()
        print("DNS server stopped")
    except Exception as e:
        print("DNS server error:", e)

def web_server():
    global gb_app_running
    try:
        addr_info = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(addr_info)
        sock.listen(5)
        sock.settimeout(1)
        print("Web server listening on port 80")
        
        while gb_app_running:
            try:
                client_sock, client_addr = sock.accept()
            except OSError:
                continue

            try:
                req = client_sock.recv(2048)
                req_str = req.decode("utf-8")
                print("HTTP request from", client_addr, ":", req_str.splitlines()[0])
                
                # Check for captive portal connectivity check (Android)
                if "generate_204" in req_str or "connectivity" in req_str or "ncsi.txt" in req_str:
                    # Redirect connectivity checks to the captive portal page.
                    ip = gcl_access_point.ifconfig()[0]
                    response = ("HTTP/1.1 302 Found\r\n"
                                "Location: http://%s/\r\n"
                                "Cache-Control: no-cache\r\n"
                                "\r\n" % ip)
                    client_sock.send(response.encode("utf-8"))
                    client_sock.close()
                    continue
                
                first_line = req_str.split("\r\n")[0]
                parts = first_line.split(" ")
                path = parts[1] if len(parts) >= 2 else "/"
                
                if path.startswith("/update?"):
                    # Parse query parameters.
                    query_string = path[len("/update?"):]
                    params = {}
                    for param in query_string.split("&"):
                        if "=" in param:
                            k, v = param.split("=", 1)
                            params[k] = v
                    x_val = float(params.get("x", "0"))
                    y_val = float(params.get("y", "0"))
                    update_speed(y_val, x_val)
                    
                    evt = params.get("evt", "none")
                    print("Event:", evt, "| Arrow command - x:", x_val, "y:", y_val)
                    response_body = "OK"
                    response = ("HTTP/1.1 200 OK\r\n"
                                "Content-Type: text/plain\r\n\r\n" + response_body)
                    client_sock.send(response.encode("utf-8"))
                    
                elif path == "/stop":
                    print("Stop command received. Stopping application...")
                    gb_app_running = False
                    response_body = ("<html><head><title>Stopped</title></head>"
                                     "<body><h1>Application Stopped</h1></body></html>")
                    response = ("HTTP/1.1 200 OK\r\n"
                                "Content-Type: text/html\r\n\r\n" + response_body)
                    client_sock.send(response.encode("utf-8"))
                
                else:
                    # Serve the captive portal page by loading the external HTML file.
                    try:
                        with open("page_arrow_control.html", "r") as f:
                            html = f.read()
                    except Exception as e:
                        print("Error loading page_arrow_control.html:", e)
                        html = ("<html><head><title>Error</title></head>"
                                "<body><h1>Error: Could not load page_arrow_control.html</h1></body></html>")
                    
                    response = ("HTTP/1.1 200 OK\r\n"
                                "Content-Type: text/html\r\n\r\n" + html)
                    client_sock.send(response.encode("utf-8"))
                
                client_sock.close()
            except Exception as e:
                print("Error handling client:", e)
        sock.close()
        print("Web server stopped")
    except Exception as e:
        print("Web server error:", e)

# --- Main code ---


# Global flag for application running.
gb_app_running = True

# The access point object (will be set later).
gcl_access_point = None

# Create a shared dictionary to store speed values, protected by a lock.
_speed_lock = _thread.allocate_lock()
_speed_data = {"forward": 0.0, "right": 0.0}

gcl_access_point = setup_access_point()

cl_alvik = ArduinoAlvik()
cl_alvik.begin()

# Start the DNS and HTTP server threads.
_thread.start_new_thread(dns_server, ())
_thread.start_new_thread(web_server, ())

# Main loop: update wheels based on shared speed data.
while gb_app_running:
    n_speed_forward, n_speed_turn = get_speed()
    n_speed_left  = - n_speed_forward * 100.0 - n_speed_turn * 100.0
    n_speed_right = - n_speed_forward * 100.0 + n_speed_turn * 100.0
    
    print(f"Move L{n_speed_left} R{n_speed_right}")
    cl_alvik.set_wheels_speed(n_speed_right, n_speed_left)
    
    sleep_ms(100)

# On exit, stop the wheels and disable the AP.
cl_alvik.set_wheels_speed(0.0, 0.0)
gcl_access_point.active(False)

print("Application has been stopped.")
