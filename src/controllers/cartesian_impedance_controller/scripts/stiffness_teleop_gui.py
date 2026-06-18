#!/usr/bin/env python3
"""GUI teleop for the Cartesian impedance controller stiffness.

Shows a window with a live bar for each of the 6 stiffness axes and
increase/decrease buttons. Button presses (and keyboard keys) publish a
single-character std_msgs/String command to the controller, which clamps
and echoes the result back on /current_stiffness -- the bars then update,
and the RViz ellipsoids move accordingly.
"""
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped

# Axis definition: (label, '+' key, '-' key, min, max)
# Keys MUST match cartesian_impedance_controller::stiffnessCommandCallback.
AXES = [
    ("Tx  [N/m]",  "a", "s",  50.0, 1000.0),
    ("Ty  [N/m]",  "d", "f",  50.0, 1000.0),
    ("Tz  [N/m]",  "w", "e",  50.0, 1000.0),
    ("Rx  [Nm/rad]", "t", "g", 10.0,  200.0),
    ("Ry  [Nm/rad]", "y", "h", 10.0,  200.0),
    ("Rz  [Nm/rad]", "o", "i", 10.0,  200.0),
]


class StiffnessTeleopGui(Node):
    def __init__(self):
        super().__init__("stiffness_teleop_gui")
        self.declare_parameter(
            "controller_ns", "/lbr/cartesian_impedance_controller")
        ns = self.get_parameter("controller_ns").value

        self.pub = self.create_publisher(String, ns + "/stiffness_command", 10)
        self.sub = self.create_subscription(
            WrenchStamped, ns + "/current_stiffness", self._on_stiffness, 10)

        self.values = [0.0] * 6        # latest stiffness, index 0..5
        self._dirty = False            # set when a new message arrives

    def send(self, key: str):
        msg = String()
        msg.data = key
        self.pub.publish(msg)

    def _on_stiffness(self, msg: WrenchStamped):
        self.values = [
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ]
        self._dirty = True


def main():
    rclpy.init()
    node = StiffnessTeleopGui()

    # Spin rclpy in a background thread so the Tk mainloop stays responsive.
    spin = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    root = tk.Tk()
    root.title("Cartesian Impedance Stiffness Teleop")
    root.geometry("460x360")

    ttk.Label(root, text="Stiffness (live)", font=("TkDefaultFont", 12, "bold")
              ).grid(row=0, column=0, columnspan=4, pady=(10, 6))

    bars, value_lbls = [], []
    for i, (label, kp, km, lo, hi) in enumerate(AXES):
        r = i + 1
        ttk.Label(root, text=label, width=12).grid(row=r, column=0, padx=6, pady=4)
        bar = ttk.Progressbar(root, length=180, maximum=hi - lo)
        bar.grid(row=r, column=1, padx=4)
        bars.append(bar)
        vlbl = ttk.Label(root, text="--", width=7)
        vlbl.grid(row=r, column=2, padx=4)
        value_lbls.append(vlbl)
        btns = tk.Frame(root)
        btns.grid(row=r, column=3)
        tk.Button(btns, text="-", width=2,
                  command=lambda k=km: node.send(k)).pack(side="left")
        tk.Button(btns, text="+", width=2,
                  command=lambda k=kp: node.send(k)).pack(side="left")
        # Keyboard shortcuts
        root.bind(kp, lambda e, k=kp: node.send(k))
        root.bind(km, lambda e, k=km: node.send(k))

    hint = ("Keys:  a/s Tx   d/f Ty   w/e Tz   t/g Rx   y/h Ry   o/i Rz\n"
            "Green ellipsoid = translational, Blue = rotational")
    ttk.Label(root, text=hint, foreground="#555").grid(
        row=8, column=0, columnspan=4, pady=(12, 4))

    def refresh():
        if node._dirty:
            node._dirty = False
            for i, (label, kp, km, lo, hi) in enumerate(AXES):
                v = node.values[i]
                bars[i]["value"] = max(0.0, min(v - lo, hi - lo))
                value_lbls[i]["text"] = f"{v:.1f}"
        if rclpy.ok():
            root.after(50, refresh)

    def on_close():
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    # Close button to stop the teleop cleanly when done
    tk.Button(root, text="Close", width=10, command=on_close).grid(
        row=9, column=0, columnspan=4, pady=(4, 10))
    root.bind("q", lambda e: on_close())

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, refresh)
    root.mainloop()


if __name__ == "__main__":
    main()
