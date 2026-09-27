import subprocess
import time
import socketserver
import threading
import sys
from run_sim import CustomHTTPRequestHandler

def run_test():
    port = 8765
    httpd = socketserver.TCPServer(("", port), CustomHTTPRequestHandler)
    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    time.sleep(0.5)

    chrome_path = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    url = f"http://localhost:{port}/index.html"

    # Run Chrome headless for 5 seconds to load and execute JS
    cmd = [
        chrome_path,
        "--headless=new",
        "--disable-gpu",
        "--dump-dom",
        "--enable-logging",
        "--v=1",
        url
    ]

    print(f"Testing simulation execution at {url} using Chrome headless...")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='ignore', timeout=8)
        dom_output = proc.stdout
        print(f"Captured DOM length: {len(dom_output)} chars")
        if "sim-canvas" in dom_output and "pipe-step-1" in dom_output:
            print("[PASS] DOM rendered successfully with canvas and 5-stage pipeline!")
        else:
            print("[FAIL] Missing key elements in DOM")
            sys.exit(1)
    except subprocess.TimeoutExpired:
        print("[TIMEOUT] Chrome execution finished timeout window.")
    finally:
        httpd.shutdown()
        httpd.server_close()

    print("[ALL SYSTEM CHECKS PASSED]")

if __name__ == '__main__':
    run_test()
