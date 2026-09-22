#!/usr/bin/env python3
"""Verify a FOTA binary URL, then publish it to a device's update topic.

Usage:
    python fota_push.py <imei> <url>
    python fota_push.py <imei> <url> --check-only

Publishes with retain=False -- ALWAYS. A retained FOTA message re-fires the
download on every reboot, which is the one loop the firmware cannot break out
of on its own.
"""
import sys, ssl, urllib.request

BROKER, PORT = "dev-fuelsense.iotronsys.com", 8777
USER, PASS   = "iotrondev", "iotrondev@1234"
BASE         = "fuelsense"

def check(url):
    print(f"Checking {url}")
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            data = r.read()
            print(f"  HTTP {r.status}   {len(data)} bytes")
    except Exception as e:
        print(f"  FAILED: {e}")
        return None
    if len(data) < 100000:
        print("  !! Suspiciously small for a firmware image -- stored as text/LFS?")
        return None
    # ESP32 app image magic byte
    if data[0] != 0xE9:
        print(f"  !! First byte is 0x{data[0]:02X}, expected 0xE9 (ESP32 image magic).")
        print("     This is NOT a valid ESP-IDF app image -- wrong file, or corrupted.")
        return None
    print("  OK: 0xE9 image magic present, size plausible.")
    return len(data)

def publish(imei, url):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("\npaho-mqtt not installed. Run:")
        print("    pip install paho-mqtt")
        sys.exit(1)
    topic = f"{BASE}/{imei}/update"
    c = mqtt.Client(client_id=f"fota-push-{imei}")
    c.username_pw_set(USER, PASS)
    c.tls_set(cert_reqs=ssl.CERT_REQUIRED)   # broker is Let's Encrypt -> system roots
    print(f"\nConnecting to {BROKER}:{PORT} ...")
    c.connect(BROKER, PORT, 30)
    c.loop_start()
    info = c.publish(topic, url, qos=1, retain=False)   # retain MUST stay False
    info.wait_for_publish(30)
    c.loop_stop(); c.disconnect()
    print(f"Published to {topic}")
    print(f"  payload: {url}")
    print("  retain : False")
    print("\nWatch the monitor for:")
    print("  HTTP 200 OK (462848 bytes) -> UFS download OK -> flashing -> reboot")
    print("  then: App version: 1.0.1")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    imei, url = sys.argv[1], sys.argv[2]
    if check(url) is None:
        print("\nAborting -- fix the upload before firing FOTA.")
        sys.exit(1)
    if "--check-only" in sys.argv:
        print("\n--check-only: not publishing."); sys.exit(0)
    publish(imei, url)
