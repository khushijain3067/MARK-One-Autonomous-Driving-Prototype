#!/usr/bin/env python3
"""
SIH 2026 - Problem Statement ID: SIH26037
Team MARK-One: Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

Local Interactive Simulation Launcher
Runs a lightweight zero-dependency HTTP server and opens the simulation in your browser.
"""

import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8000

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # Enable caching controls and CORS for local testing
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

    def guess_type(self, path):
        # Ensure proper MIME type for ES modules
        if path.endswith('.js'):
            return 'application/javascript'
        return super().guess_type(path)

def run_server():
    # Change working directory to current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    handler = CustomHTTPRequestHandler
    
    # Attempt to bind to PORT or increment if occupied
    port = PORT
    max_attempts = 10
    httpd = None

    for attempt in range(max_attempts):
        try:
            httpd = socketserver.TCPServer(("", port), handler)
            break
        except OSError:
            print(f"[!] Port {port} is occupied, trying port {port + 1}...")
            port += 1

    if not httpd:
        print("[X] Error: Could not bind to an open port.")
        sys.exit(1)

    url = f"http://localhost:{port}/index.html"
    print("=" * 70)
    print(" SMART INDIA HACKATHON 2026 - PROBLEM STATEMENT: SIH26037")
    print(" Team MARK-One: Autonomous Driving on Unstructured Indian Roads")
    print("=" * 70)
    print(f"[+] Simulation server running at: {url}")
    print("[+] Opening simulation in default web browser...")
    print("[+] Press Ctrl+C in this terminal to stop the server.")
    print("=" * 70)

    try:
        webbrowser.open(url)
    except Exception as e:
        print(f"[!] Could not open browser automatically: {e}")
        print(f"[!] Please open your browser manually and visit: {url}")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[+] Shutting down simulation server. Exiting gracefully.")
        httpd.server_close()

if __name__ == '__main__':
    run_server()
