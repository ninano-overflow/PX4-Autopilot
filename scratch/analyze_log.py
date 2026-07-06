import sys
from pyulog import ULog
import numpy as np

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_log.py <path_to_ulog>")
        return

    ulog = ULog(sys.argv[1])
    
    try:
        lpos = ulog.get_dataset('vehicle_local_position')
        t = lpos.data['timestamp'] / 1e6
        vx = lpos.data['vx']
        vy = lpos.data['vy']
        vz = lpos.data['vz']
        
        # Also get attitude for correlation
        att = ulog.get_dataset('vehicle_attitude')
        t_att = att.data['timestamp'] / 1e6
        q_w = att.data['q[0]']
        q_x = att.data['q[1]']
        q_y = att.data['q[2]']
        q_z = att.data['q[3]']
        
        def quaternion_to_euler(w, x, y, z):
            sinp = 2 * (w * y - z * x)
            if np.abs(sinp) >= 1:
                pitch = np.copysign(np.pi / 2, sinp)
            else:
                pitch = np.arcsin(sinp)
            return np.degrees(pitch)
            
        print("Time (s) | vx (m/s) | vy (m/s) | vz (m/s) | Speed (m/s) | Pitch (deg)")
        print("-" * 75)
        for i in range(len(t)):
            if 35.0 <= t[i] <= 45.0:
                # Find closest attitude index
                idx_att = np.argmin(np.abs(t_att - t[i]))
                p = quaternion_to_euler(q_w[idx_att], q_x[idx_att], q_y[idx_att], q_z[idx_att])
                speed = np.sqrt(vx[i]**2 + vy[i]**2 + vz[i]**2)
                print(f"{t[i]:8.2f} | {vx[i]:8.2f} | {vy[i]:8.2f} | {vz[i]:8.2f} | {speed:11.2f} | {p:10.2f}")
    except Exception as e:
        print("Error reading vehicle_local_position:", e)

if __name__ == '__main__':
    main()
