import subprocess
import time
import socketserver
import threading
import sys
import re
from run_sim import CustomHTTPRequestHandler

def test_browser_scenarios():
    port = 8791
    httpd = socketserver.TCPServer(("", port), CustomHTTPRequestHandler)
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    time.sleep(0.5)

    chrome_path = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    
    scenarios_to_test = [
        ('normal_traffic', 'Scenario 1: Normal Mixed Traffic', 32.0),
        ('slow_vehicle', 'Scenario 2: Slow Vehicle Overtake', 32.0),
        ('pedestrian_crossing', 'Scenario 3: Pedestrian Crossing', 25.0),
        ('pothole_avoidance', 'Scenario 4: Pothole Avoidance', 34.0),
        ('parked_vehicle', 'Scenario 5: Parked Vehicle Bypass', 32.0),
        ('narrow_road', 'Scenario 8: Narrow Chokepoint & Barricades', 30.0),
        ('cattle_crossing', 'Scenario 9: Cattle Crossing Event', 25.0),
    ]

    print("==================================================")
    print("MARK-ONE SIH 2026: MULTI-SCENARIO BROWSER VERIFICATION")
    print("==================================================")

    for sc_id, sc_name, min_expected_speed in scenarios_to_test:
        url = f"http://localhost:{port}/index.html?scenario={sc_id}"
        cmd = [
            chrome_path,
            "--headless=new",
            "--disable-gpu",
            "--dump-dom",
            url
        ]

        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='ignore', timeout=15)
            dom = proc.stdout

            speed_match = re.search(r'id="val-speed"[^>]*>([^<]+)<', dom)
            dist_match = re.search(r'id="val-dist"[^>]*>([^<]+)<', dom)
            mode_match = re.search(r'id="val-mode"[^>]*>([^<]+)<', dom)
            decision_match = re.search(r'id="val-plan-decision"[^>]*>([^<]+)<', dom)
            status_match = re.search(r'id="val-plan-status"[^>]*>([^<]+)<', dom)

            speed_val = speed_match.group(1).strip() if speed_match else "UNKNOWN"
            dist_val = dist_match.group(1).strip() if dist_match else "UNKNOWN"
            mode_val = mode_match.group(1).strip() if mode_match else "UNKNOWN"
            decision_val = decision_match.group(1).strip() if decision_match else "UNKNOWN"
            status_val = status_match.group(1).strip() if status_match else "UNKNOWN"

            print(f"\n--- {sc_name} ---")
            print(f"  Speed: {speed_val} km/h | Dist: {dist_val} | Mode: {mode_val}")
            print(f"  Status: {status_val} | Decision: {decision_val}")

            speed_num = float(speed_val) if speed_val.replace('.', '', 1).isdigit() else 0.0
            dist_num = float(dist_val.replace('m', '').strip()) if dist_val.replace('m', '').strip().replace('.', '', 1).isdigit() else 0.0

            assert speed_num >= min_expected_speed, f"Speed ({speed_num} km/h) dropped below min expected {min_expected_speed} km/h"
            assert status_val != "BLOCKED_STOP", f"Vehicle incorrectly stopped with BLOCKED_STOP"
            assert dist_num > 0.5, f"Vehicle failed to make forward progress (dist={dist_num}m)"
            print(f"  >>> PASS: Speed {speed_val} km/h >= {min_expected_speed} km/h, dist {dist_val}, safe path active")

        except Exception as e:
            print(f"  [ERROR in {sc_name}]: {e}")
            httpd.shutdown()
            httpd.server_close()
            sys.exit(1)

    print("\n==================================================")
    print("ALL SCENARIO BROWSER TESTS PASSED SUCCESSFULLY!")
    print("==================================================")
    httpd.shutdown()
    httpd.server_close()

if __name__ == '__main__':
    test_browser_scenarios()
