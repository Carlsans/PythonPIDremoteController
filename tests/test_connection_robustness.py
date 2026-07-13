"""Integration test: run the real yogurtdata.py headless against a fake
ESP8266 and verify it survives malformed packets and connection loss.

Scenario:
1. normal traffic for a few seconds,
2. a barrage of malformed/corrupted UDP packets (these used to crash the
   program, taking the graph down with it),
3. silence longer than the connection timeout (simulates the MCU dropping
   off the network) - the program must reset its socket and keep running,
4. normal traffic again.

The process must still be alive at the end, with no traceback in its output.

Run from the project root:
    ./venvarch/bin/python tests/test_connection_robustness.py
"""

import os
import socket
import subprocess
import sys
import tempfile
import time

PROJECTROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHON = os.path.join(PROJECTROOT, "venvarch", "bin", "python")
# Isolated loopback ports: the program under test listens on LISTENPORT and
# sends its commands to the fake ESP on ESPPORT. It must NEVER use the real
# port 5000 / real ESP IP, or the test would command the real fermenter.
LISTENPORT = 55000
ESPPORT = 55001


def send(sock, payload):
    if isinstance(payload, str):
        payload = payload.encode()
    sock.sendto(payload, ("127.0.0.1", LISTENPORT))


def drain(sock):
    """Discard commands the program sent to the fake ESP."""
    try:
        while True:
            sock.recvfrom(4096)
    except (BlockingIOError, socket.timeout):
        pass


def normaltraffic(sock, seconds, basetemp=25.0):
    for i in range(seconds):
        send(sock, "Sensor temp: " + str(basetemp + i * 0.25))
        send(sock, "Output:127.0")
        send(sock, "PID Terms: 1.5,0.2,0.0")
        send(sock, "SetPoint:39.0")
        drain(sock)
        time.sleep(1)


def main():
    rundir = tempfile.mkdtemp(prefix="yogurttest")  # keeps the CSV out of the repo
    env = dict(os.environ)
    env["MPLBACKEND"] = "Agg"  # headless graph
    env["PYTHONUNBUFFERED"] = "1"  # keep stdout live so terminate() doesn't lose it
    env["YOGURT_MODE"] = "pidprogram"
    env["YOGURT_CONNECTION_TIMEOUT"] = "6"
    env["PYTHONPATH"] = PROJECTROOT
    env["YOGURT_ESP_IP"] = "127.0.0.1"
    env["YOGURT_ESP_PORT"] = str(ESPPORT)
    env["YOGURT_LISTEN_PORT"] = str(LISTENPORT)

    proc = subprocess.Popen(
        [PYTHON, os.path.join(PROJECTROOT, "src", "yogurtdata.py")],
        cwd=rundir, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    # The fake ESP: receives the program's commands and sends it status.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", ESPPORT))
    sock.setblocking(False)
    try:
        time.sleep(2)  # let it bind and open the (headless) figure
        assert proc.poll() is None, "process died on startup"

        print("Phase 1: normal traffic...")
        normaltraffic(sock, 4)

        print("Phase 2: malformed packets...")
        for junk in [b"\xff\xfe\xfa\x00garbage",
                     "Sensor temp: abc",
                     "Sensor temp: ",
                     "PID Terms: 1.2,,3",
                     "PID Terms: 5",
                     "Output:xyz",
                     "PIDsettingsagg(Kp,Ki,Kd):1,2",
                     "PIDsettingscons(Kp,Ki,Kd):a,b,c",
                     "SetPoint:",
                     ""]:
            send(sock, junk)
        time.sleep(2)
        assert proc.poll() is None, "process died on malformed packets"

        print("Phase 3: normal traffic after junk...")
        normaltraffic(sock, 2, basetemp=26.0)
        assert proc.poll() is None, "process died after junk phase"

        print("Phase 4: connection loss (silence > timeout)...")
        time.sleep(9)
        assert proc.poll() is None, "process died during connection loss"

        print("Phase 5: connection restored...")
        normaltraffic(sock, 3, basetemp=27.0)
        assert proc.poll() is None, "process died after reconnection"
    finally:
        sock.close()
        proc.terminate()
        try:
            output = proc.communicate(timeout=10)[0]
        except subprocess.TimeoutExpired:
            proc.kill()
            output = proc.communicate()[0]

    assert "Traceback" not in output, "process raised an exception:\n" + output
    assert "Ignoring malformed message" in output or "Ignoring undecodable packet" in output, \
        "malformed packets were not detected:\n" + output
    assert "No packet received for" in output, \
        "connection loss was not detected:\n" + output
    # Temperatures from phase 5 prove it still processes data after the reset.
    assert "Sensor temp: 27" in output, \
        "no data processed after reconnection:\n" + output

    print("\nOK - survives malformed packets and connection loss, keeps running.")


if __name__ == '__main__':
    main()
