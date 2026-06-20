#!/usr/bin/env python3
"""Plot perch errors, TOF, and OptiTrack position from a rosbag to PNGs (headless-friendly).

Usage:
    rosrun pearch_mission plot_perch_bag.py [BAG] [OUT_DIR]

Produces in OUT_DIR:
    perch_errors.png    - error_x / error_y vs time
    perch_tof.png       - TOF distance vs time
    optitrack_xyz.png   - OptiTrack x / y / z vs time
    perch_overview.png  - all of the above stacked, shared time axis
"""

import os
import sys
import rosbag
import matplotlib
matplotlib.use('Agg')  # headless: render to file, no X display needed
import matplotlib.pyplot as plt

DEFAULT_BAG = '/root/uav_ws/src/pearch_mission/bags/2026-06-12-11-34-40.bag'
DEFAULT_OUT = '/root/uav_ws/src/pearch_mission/plot'

ODOM_TOPIC = '/red/vrpn_client/estimated_odometry'


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG
    out_dir = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT
    os.makedirs(out_dir, exist_ok=True)

    ex_t, ex = [], []
    ey_t, ey = [], []
    tof_t, tof = [], []
    ot_t, ox, oy, oz = [], [], [], []

    with rosbag.Bag(bag_path) as bag:
        t0 = bag.get_start_time()
        for topic, msg, t in bag.read_messages(topics=[
                '/perch/error_x', '/perch/error_y', '/perch/tof', ODOM_TOPIC]):
            ts = t.to_sec() - t0
            if topic == '/perch/error_x':
                ex_t.append(ts); ex.append(msg.data)
            elif topic == '/perch/error_y':
                ey_t.append(ts); ey.append(msg.data)
            elif topic == '/perch/tof':
                tof_t.append(ts); tof.append(msg.data)
            else:
                p = msg.pose.pose.position
                ot_t.append(ts); ox.append(p.x); oy.append(p.y); oz.append(p.z)

    # --- 1) Errors ---
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(ex_t, ex, label='error_x', color='tab:red')
    ax.plot(ey_t, ey, label='error_y', color='tab:orange')
    ax.axhline(0, color='k', lw=0.5, ls=':')
    ax.set_xlabel('time [s]'); ax.set_ylabel('pixel error')
    ax.set_title('Perch alignment errors (x, y)')
    ax.legend(loc='best'); ax.grid(True)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'perch_errors.png'), dpi=120)
    plt.close(fig)

    # --- 2) TOF ---
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(tof_t, tof, label='tof', color='tab:green')
    ax.set_xlabel('time [s]'); ax.set_ylabel('distance')
    ax.set_title('Perch TOF distance')
    ax.legend(loc='best'); ax.grid(True)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'perch_tof.png'), dpi=120)
    plt.close(fig)

    # --- 3) OptiTrack x/y/z ---
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(ot_t, ox, label='opti x', color='tab:blue')
    ax.plot(ot_t, oy, label='opti y', color='tab:green')
    ax.plot(ot_t, oz, label='opti z', color='tab:purple')
    ax.set_xlabel('time [s]'); ax.set_ylabel('position [m]')
    ax.set_title('OptiTrack position (vrpn estimated_odometry)')
    ax.legend(loc='best'); ax.grid(True)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'optitrack_xyz.png'), dpi=120)
    plt.close(fig)

    # --- 4) Overview (shared time axis) ---
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    axes[0].plot(ex_t, ex, label='error_x', color='tab:red')
    axes[0].plot(ey_t, ey, label='error_y', color='tab:orange')
    axes[0].axhline(0, color='k', lw=0.5, ls=':')
    axes[0].set_ylabel('pixel error'); axes[0].set_title('Perch errors')
    axes[0].legend(loc='best'); axes[0].grid(True)

    axes[1].plot(tof_t, tof, label='tof', color='tab:green')
    axes[1].set_ylabel('distance'); axes[1].set_title('TOF')
    axes[1].legend(loc='best'); axes[1].grid(True)

    axes[2].plot(ot_t, ox, label='opti x', color='tab:blue')
    axes[2].plot(ot_t, oy, label='opti y', color='tab:green')
    axes[2].plot(ot_t, oz, label='opti z', color='tab:purple')
    axes[2].set_ylabel('position [m]'); axes[2].set_xlabel('time [s]')
    axes[2].set_title('OptiTrack x/y/z')
    axes[2].legend(loc='best'); axes[2].grid(True)

    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'perch_overview.png'), dpi=120)
    plt.close(fig)

    # --- 5) Top-down XY trajectory (perch path) ---
    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(ox, oy, c=ot_t, cmap='viridis', s=6)
    ax.plot(ox, oy, color='gray', lw=0.4, alpha=0.5)
    if ox:
        ax.scatter([ox[0]], [oy[0]], color='lime', s=80, marker='o', label='start', zorder=5)
        ax.scatter([ox[-1]], [oy[-1]], color='red', s=80, marker='X', label='end', zorder=5)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('OptiTrack top-down trajectory (color = time)')
    ax.axis('equal'); ax.grid(True); ax.legend(loc='best')
    fig.colorbar(sc, ax=ax, label='time [s]')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'optitrack_xy_path.png'), dpi=120)
    plt.close(fig)

    print(f"Saved plots to {out_dir}")
    print(f"  error_x: {len(ex)} pts, error_y: {len(ey)} pts, "
          f"tof: {len(tof)} pts, optitrack: {len(ox)} pts")


if __name__ == '__main__':
    main()
