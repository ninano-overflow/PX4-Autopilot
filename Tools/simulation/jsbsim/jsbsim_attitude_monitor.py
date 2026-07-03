#!/usr/bin/env python3
"""
JSBSim attitude monitor — listens on UDP port 5762 for JSBSim socket output.
JSBSim sends UDP packets to this script (client=JSBSim, server=this script).

Usage:
    python3 jsbsim_attitude_monitor.py   # start BEFORE or AFTER the simulation
"""

import socket

HOST = "0.0.0.0"
PORT = 5762


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((HOST, PORT))
        print(f"Listening for JSBSim UDP output on port {PORT} ...\n")
        buf = ""
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                buf += data.decode("utf-8", errors="replace")
                lines = buf.split("\n")
                buf = lines[-1]  # keep incomplete last line
                for line in lines[:-1]:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith("Time"):
                        print(f"[HEADER] {line}")
                        continue
                    parts = line.split(",")
                    if len(parts) >= 9:
                        t, phi, theta, psi, alt, vt, ax, ay, az = parts[:9]
                        print(
                            f"t={float(t):7.2f}s  "
                            f"phi={float(phi):6.2f}°  "
                            f"theta={float(theta):6.2f}°  "
                            f"psi={float(psi):7.2f}°  "
                            f"ax={float(ax):7.2f}  ay={float(ay):7.2f}  az={float(az):7.2f}"
                        )
                    elif len(parts) >= 6:
                        t, phi, theta, psi, alt, vt = parts[:6]
                        print(
                            f"t={float(t):7.2f}s  "
                            f"phi={float(phi):7.2f}°  "
                            f"theta={float(theta):7.2f}°  "
                            f"psi={float(psi):7.2f}°  "
                            f"alt={float(alt):6.1f}m  "
                            f"vt={float(vt):5.1f}fps"
                        )
            except KeyboardInterrupt:
                print("\nStopped.")
                break


if __name__ == "__main__":
    main()