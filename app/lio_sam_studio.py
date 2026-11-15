#!/usr/bin/env python3
"""LIO-SAM Studio: a single desktop entry point for the survey and recording GUIs."""
import os
import sys
import tkinter as tk
from tkinter import messagebox

APP_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.dirname(APP_DIR)
sys.path.insert(0, WORKSPACE_DIR)

from lio_sam_gui import LIOSAMApp  # noqa: E402
from lio_sam_record_gui import RecorderGUI  # noqa: E402

BG = "#1e2329"
CARD = "#2a3038"
CARD_HOVER = "#343b45"
FG = "#e8eaed"
MUTED = "#9aa0a6"
ACCENT = "#26a69a"


class StudioHub:
    def __init__(self, root):
        self.root = root
        root.title("LIO-SAM Studio")
        root.geometry("560x420")
        root.minsize(480, 380)
        root.configure(bg=BG)

        icon_png = os.path.join(APP_DIR, "icons", "lio-sam-studio.png")
        if os.path.exists(icon_png):
            try:
                self._icon = tk.PhotoImage(file=icon_png)
                root.iconphoto(True, self._icon)
            except tk.TclError:
                pass

        self.windows = {}  # name -> (toplevel, app)

        tk.Label(root, text="LIO-SAM Studio", bg=BG, fg=FG,
                 font=("Sans", 20, "bold")).pack(pady=(28, 4))
        tk.Label(root, text="LiDAR-inertial survey mapping", bg=BG, fg=MUTED,
                 font=("Sans", 11)).pack(pady=(0, 22))

        self._card("🔴  Live Recording",
                   "Record LiDAR + IMU topics to a ROS 2 bag",
                   lambda: self.open_tool("record", RecorderGUI))
        self._card("🗺  Survey Processing",
                   "Run SLAM on a bag, save the map, export PLY / LAS",
                   lambda: self.open_tool("survey", LIOSAMApp))

        ros_ok = bool(os.environ.get("ROS_DISTRO"))
        status = f"ROS 2 {os.environ.get('ROS_DISTRO')} environment loaded" if ros_ok \
            else "⚠ ROS 2 not sourced: start via lio_sam_studio.sh"
        tk.Label(root, text=status, bg=BG, fg=ACCENT if ros_ok else "#ef5350",
                 font=("Sans", 9)).pack(side=tk.BOTTOM, pady=12)

        root.protocol("WM_DELETE_WINDOW", self.quit)

    def _card(self, title, subtitle, command):
        card = tk.Frame(self.root, bg=CARD, cursor="hand2", padx=18, pady=14)
        card.pack(fill=tk.X, padx=36, pady=8)
        t = tk.Label(card, text=title, bg=CARD, fg=FG, font=("Sans", 14, "bold"), anchor="w")
        t.pack(fill=tk.X)
        s = tk.Label(card, text=subtitle, bg=CARD, fg=MUTED, font=("Sans", 10), anchor="w")
        s.pack(fill=tk.X)
        widgets = (card, t, s)

        def set_bg(color):
            for w in widgets:
                w.configure(bg=color)

        for w in widgets:
            w.bind("<Button-1>", lambda _e: command())
            w.bind("<Enter>", lambda _e: set_bg(CARD_HOVER))
            w.bind("<Leave>", lambda _e: set_bg(CARD))

    def open_tool(self, name, cls):
        if name in self.windows:
            top, _ = self.windows[name]
            top.deiconify()
            top.lift()
            top.focus_force()
            return
        top = tk.Toplevel(self.root)
        app = cls(top)
        self.windows[name] = (top, app)
        top.protocol("WM_DELETE_WINDOW", lambda: self.close_tool(name))

    def _busy(self, app):
        if isinstance(app, RecorderGUI):
            return app.is_recording
        return bool(app.lio_process or app.bag_process)

    def _shutdown(self, app):
        if isinstance(app, RecorderGUI):
            app.stop_recording()
        else:
            app.stop_all()

    def close_tool(self, name):
        top, app = self.windows[name]
        if self._busy(app):
            if not messagebox.askyesno("Still running",
                                       "Processes are still running in this window.\n"
                                       "Stop them and close?", parent=top):
                return
            self._shutdown(app)
        top.destroy()
        del self.windows[name]

    def quit(self):
        busy = [n for n, (_, a) in self.windows.items() if self._busy(a)]
        if busy and not messagebox.askyesno(
                "Quit LIO-SAM Studio",
                "Recording or SLAM is still running.\nStop everything and quit?"):
            return
        for _, app in self.windows.values():
            try:
                self._shutdown(app)
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk(className="lio-sam-studio")  # WM_CLASS matches StartupWMClass
    StudioHub(root)
    root.mainloop()


if __name__ == "__main__":
    main()
