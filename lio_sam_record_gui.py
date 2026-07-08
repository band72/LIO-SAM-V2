#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import subprocess
import threading
import os
import time

class RecorderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LIO-SAM Bag Recorder")
        self.root.geometry("700x600")
        self.root.configure(padx=10, pady=10)
        
        self.recording_process = None
        self.is_recording = False
        self.workspace_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.setup_ui()
        self.refresh_topics()

    def setup_ui(self):
        # --- Top Section: Output Path ---
        frame_out = tk.LabelFrame(self.root, text="Output Destination", padx=10, pady=10)
        frame_out.pack(fill=tk.X, pady=5)
        
        tk.Label(frame_out, text="Bag Name/Folder:").grid(row=0, column=0, sticky="w")
        self.bag_name_var = tk.StringVar(value=os.path.join(self.workspace_dir, "my_survey_bag"))
        tk.Entry(frame_out, textvariable=self.bag_name_var, width=50).grid(row=0, column=1, padx=5)
        tk.Button(frame_out, text="Browse", command=self.browse_out).grid(row=0, column=2)
        
        # --- Middle Section: Sensor Config ---
        frame_sensors = tk.LabelFrame(self.root, text="Sensor Configuration", padx=10, pady=10)
        frame_sensors.pack(fill=tk.X, pady=5)
        
        tk.Label(frame_sensors, text="LiDAR Topic:").grid(row=0, column=0, sticky="w", pady=2)
        self.lidar_var = tk.StringVar(value="/points_raw")
        tk.Entry(frame_sensors, textvariable=self.lidar_var, width=30).grid(row=0, column=1, padx=5, sticky="w")
        
        tk.Label(frame_sensors, text="IMU Topic:").grid(row=1, column=0, sticky="w", pady=2)
        self.imu_var = tk.StringVar(value="/imu_correct")
        tk.Entry(frame_sensors, textvariable=self.imu_var, width=30).grid(row=1, column=1, padx=5, sticky="w")
        
        # --- Live Topics Pane ---
        frame_topics = tk.LabelFrame(self.root, text="Live ROS 2 Topics", padx=10, pady=5)
        frame_topics.pack(fill=tk.BOTH, expand=True, pady=5)
        
        tk.Button(frame_topics, text="↻ Refresh Topics", command=self.refresh_topics).pack(anchor="ne", pady=2)
        self.topic_list = tk.Text(frame_topics, height=8, bg="#f0f0f0")
        self.topic_list.pack(fill=tk.BOTH, expand=True)
        
        # --- Controls ---
        frame_controls = tk.Frame(self.root, pady=10)
        frame_controls.pack(fill=tk.X)
        
        self.btn_record = tk.Button(frame_controls, text="🔴 Start Recording", font=("Arial", 12, "bold"), bg="#f44336", fg="white", command=self.start_recording)
        self.btn_record.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        
        self.btn_stop = tk.Button(frame_controls, text="⏹ Stop Recording", font=("Arial", 12, "bold"), bg="#9e9e9e", state=tk.DISABLED, command=self.stop_recording)
        self.btn_stop.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        
        # --- Console Output ---
        tk.Label(self.root, text="Recording Output:").pack(anchor="w")
        self.console = tk.Text(self.root, bg="black", fg="lightgreen", height=8)
        self.console.pack(fill=tk.BOTH, expand=True)

    def browse_out(self):
        path = filedialog.askdirectory(title="Select Output Directory")
        if path:
            self.bag_name_var.set(os.path.join(path, f"survey_{int(time.time())}"))

    def log(self, msg):
        self.console.insert(tk.END, msg + "\n")
        self.console.see(tk.END)
        self.root.update()

    def refresh_topics(self):
        self.topic_list.delete("1.0", tk.END)
        self.topic_list.insert(tk.END, "Querying live topics...\n")
        self.root.update()
        
        def run_query():
            try:
                out = subprocess.check_output("ros2 topic list", shell=True, text=True, stderr=subprocess.STDOUT)
                self.root.after(0, lambda: self.update_topic_display(out))
            except Exception as e:
                self.root.after(0, lambda: self.update_topic_display(f"Error querying topics: {str(e)}\nMake sure ROS 2 is sourced!"))
                
        threading.Thread(target=run_query, daemon=True).start()
        
    def update_topic_display(self, text):
        self.topic_list.delete("1.0", tk.END)
        self.topic_list.insert(tk.END, text)

    def start_recording(self):
        bag_out = self.bag_name_var.get().strip()
        lidar = self.lidar_var.get().strip()
        imu = self.imu_var.get().strip()
        
        if not bag_out or not lidar or not imu:
            messagebox.showerror("Error", "Please fill out all fields!")
            return
            
        if os.path.exists(bag_out):
            msg = f"Directory already exists:\n{bag_out}\n\nDo you want to append a timestamp?"
            if messagebox.askyesno("Directory Exists", msg):
                bag_out = f"{bag_out}_{int(time.time())}"
                self.bag_name_var.set(bag_out)
            else:
                return

        self.is_recording = True
        self.btn_record.config(state=tk.DISABLED, bg="#9e9e9e")
        self.btn_stop.config(state=tk.NORMAL, bg="#2196F3", fg="white")
        self.log(f"=> Starting recording to: {bag_out}")
        self.log(f"=> Topics: {lidar}, {imu}")
        
        cmd = f"ros2 bag record {lidar} {imu} -o {bag_out}"
        
        def run_record():
            try:
                self.recording_process = subprocess.Popen(
                    cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
                )
                for line in self.recording_process.stdout:
                    self.root.after(0, self.log, line.strip())
            except Exception as e:
                self.root.after(0, self.log, f"Error: {str(e)}")
            
            self.root.after(0, self.reset_buttons)
                
        threading.Thread(target=run_record, daemon=True).start()

    def stop_recording(self):
        if self.recording_process and self.is_recording:
            self.log("=> Sending termination signal to ros2 bag record...")
            try:
                subprocess.run("pkill -INT -f 'ros2 bag record'", shell=True)
            except Exception as e:
                self.log(f"Error stopping: {e}")
            self.is_recording = False

    def reset_buttons(self):
        self.is_recording = False
        self.btn_record.config(state=tk.NORMAL, bg="#f44336")
        self.btn_stop.config(state=tk.DISABLED, bg="#9e9e9e")
        self.log("=> Recording stopped.")

if __name__ == "__main__":
    root = tk.Tk()
    app = RecorderGUI(root)
    root.mainloop()
