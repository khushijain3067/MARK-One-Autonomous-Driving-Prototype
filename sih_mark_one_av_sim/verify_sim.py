import os
import sys
import threading
import time
import urllib.request
import http.server
import socketserver

def check_files():
    required_files = [
        "index.html",
        "css/style.css",
        "js/config.js",
        "js/models/Road.js",
        "js/models/Vehicle.js",
        "js/models/Obstacle.js",
        "js/perception/Sensors.js",
        "js/perception/SensorFusion.js",
        "js/risk/RiskAssessment.js",
        "js/planning/TrajectoryPredictor.js",
        "js/planning/LatticePlanner.js",
        "js/control/VehicleController.js",
        "js/scenarios/ScenarioManager.js",
        "js/ui/Renderer.js",
        "js/ui/Dashboard.js",
        "js/main.js",
        "run_sim.py"
    ]
    missing = []
    for f in required_files:
        if not os.path.exists(f):
            missing.append(f)
    if missing:
        print(f"FAILED: Missing files: {missing}")
        return False
    print(f"SUCCESS: All {len(required_files)} files exist on disk.")
    return True

def test_server():
    from run_sim import CustomHTTPRequestHandler
    port = 8899
    httpd = socketserver.TCPServer(("", port), CustomHTTPRequestHandler)
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    time.sleep(0.5)

    urls = [
        f"http://localhost:{port}/index.html",
        f"http://localhost:{port}/css/style.css",
        f"http://localhost:{port}/js/main.js",
        f"http://localhost:{port}/js/config.js",
        f"http://localhost:{port}/js/models/Vehicle.js",
        f"http://localhost:{port}/js/models/Road.js",
        f"http://localhost:{port}/js/models/Obstacle.js",
        f"http://localhost:{port}/js/perception/Sensors.js",
        f"http://localhost:{port}/js/perception/SensorFusion.js",
        f"http://localhost:{port}/js/risk/RiskAssessment.js",
        f"http://localhost:{port}/js/planning/TrajectoryPredictor.js",
        f"http://localhost:{port}/js/planning/LatticePlanner.js",
        f"http://localhost:{port}/js/control/VehicleController.js",
        f"http://localhost:{port}/js/scenarios/ScenarioManager.js",
        f"http://localhost:{port}/js/ui/Renderer.js",
        f"http://localhost:{port}/js/ui/Dashboard.js"
    ]

    all_ok = True
    for u in urls:
        req = urllib.request.Request(u)
        with urllib.request.urlopen(req) as resp:
            status = resp.status
            content_type = resp.headers.get('Content-Type')
            content_len = len(resp.read())
            print(f"[HTTP {status}] {u} ({content_type}, {content_len} bytes)")
            if status != 200:
                all_ok = False

    httpd.shutdown()
    httpd.server_close()
    return all_ok

if __name__ == '__main__':
    if not check_files():
        sys.exit(1)
    if not test_server():
        sys.exit(1)
    print("\nALL VERIFICATION CHECKS PASSED SUCCESSFULLY!")
