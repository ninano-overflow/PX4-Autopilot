#!/usr/bin/env python3
"""
JSBSim attitude monitor — listens on UDP port 5762 for JSBSim socket output.
JSBSim sends UDP packets to this script (client=JSBSim, server=this script).

Fields (in order from goshawk_10s.xml output block):
  time, phi, theta, psi, alt, vt,
  acc-x, acc-y, acc-z,
  gyro-x, gyro-y, gyro-z,
  biased-mag-x, biased-mag-y, biased-mag-z

Usage:
    python3 jsbsim_attitude_monitor.py   # start BEFORE or AFTER the simulation
"""

import socket

HOST = "0.0.0.0"
PORT = 5762

FIELDS = [
    "time",
    "phi", "theta", "psi",
    "alt", "vt",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "mx", "my", "mz",
]


def fmt(label, value, unit="", width=8):
    return f"{label}={value:{width}.3f}{unit}"


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
                buf = lines[-1]
                for line in lines[:-1]:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith("Time"):
                        continue
                    parts = line.split(",")
                    try:
                        vals = [float(p) for p in parts]
                    except ValueError:
                        continue

                    n = len(vals)
                    t     = vals[0]  if n > 0  else 0.0
                    phi   = vals[1]  if n > 1  else 0.0
                    theta = vals[2]  if n > 2  else 0.0
                    psi   = vals[3]  if n > 3  else 0.0
                    alt   = vals[4]  if n > 4  else 0.0
                    vt    = vals[5]  if n > 5  else 0.0
                    ax    = vals[6]  if n > 6  else 0.0
                    ay    = vals[7]  if n > 7  else 0.0
                    az    = vals[8]  if n > 8  else 0.0
                    gx    = vals[9]  if n > 9  else 0.0
                    gy    = vals[10] if n > 10 else 0.0
                    gz    = vals[11] if n > 11 else 0.0
                    mx    = vals[12] if n > 12 else 0.0
                    my    = vals[13] if n > 13 else 0.0
                    mz    = vals[14] if n > 14 else 0.0

                    print(
                        f"t={t:7.2f}s | "
                        f"phi={phi:6.1f}° theta={theta:6.1f}° psi={psi:7.1f}° | "
                        f"alt={alt:5.1f}m vt={vt:5.1f}fps | "
                        f"ax={ax:7.3f} ay={ay:7.3f} az={az:7.3f} | "
                        f"gx={gx:7.4f} gy={gy:7.4f} gz={gz:7.4f} | "
                        f"mx={mx:8.5f} my={my:8.5f} mz={mz:8.5f}"
                    )
            except KeyboardInterrupt:
                print("\nStopped.")
                break


if __name__ == "__main__":
    main()
