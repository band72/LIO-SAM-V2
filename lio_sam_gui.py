#!/usr/bin/env python3
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import subprocess
import threading
import os
import signal
import time

class LIOSAMApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LIO-SAM Survey Interface")
        self.root.geometry("800x600")
        
        # Workspace Path
        self.workspace_dir = "/home/artwalk/.gemini/antigravity-ide/scratch/LIO-SAM"
        self.map_dest = os.path.join(self.workspace_dir, "map/")
        
        self.lio_process = None
        self.bag_process = None
        
        self.setup_ui()
        
    def setup_ui(self):
        frame_top = tk.Frame(self.root, padx=10, pady=10)
        frame_top.pack(fill=tk.X)
        
        # Bag File Selection
        tk.Label(frame_top, text="ROS 2 Bag:").grid(row=0, column=0, sticky="w", pady=5)
        self.bag_path_var = tk.StringVar()
        tk.Entry(frame_top, textvariable=self.bag_path_var, width=50).grid(row=0, column=1, padx=5)
        tk.Button(frame_top, text="Browse", command=self.browse_bag).grid(row=0, column=2)
        
        # Config Selection
        tk.Label(frame_top, text="Configuration:").grid(row=1, column=0, sticky="w", pady=5)
        self.config_var = tk.StringVar(value="params.yaml")
        config_dropdown = ttk.Combobox(frame_top, textvariable=self.config_var, state="readonly", width=47)
        config_dropdown['values'] = ("params.yaml", "params_survey_grade.yaml")
        config_dropdown.grid(row=1, column=1, padx=5, sticky="w")
        tk.Button(frame_top, text="✎ Edit", command=self.edit_config).grid(row=1, column=2, padx=5)
        
        # Conversion Toggle
        self.convert_var = tk.BooleanVar(value=True)
        tk.Checkbutton(frame_top, text="Convert to PLY", variable=self.convert_var).grid(row=2, column=1, sticky="w", pady=5)
        
        self.convert_las_var = tk.BooleanVar(value=False)
        tk.Checkbutton(frame_top, text="Convert to LAS", variable=self.convert_las_var).grid(row=2, column=1, padx=120, sticky="w", pady=5)
        
        # Buttons
        frame_btns = tk.Frame(self.root, padx=10, pady=5)
        frame_btns.pack(fill=tk.X)
        
        self.btn_run = tk.Button(frame_btns, text="▶ Start SLAM", bg="#4CAF50", fg="white", command=self.start_slam)
        self.btn_run.pack(side=tk.LEFT, padx=5)
        
        self.btn_save = tk.Button(frame_btns, text="💾 Save Map & Convert", bg="#2196F3", fg="white", command=self.save_map, state=tk.DISABLED)
        self.btn_save.pack(side=tk.LEFT, padx=5)
        
        self.btn_stop = tk.Button(frame_btns, text="⏹ Stop", bg="#f44336", fg="white", command=self.stop_all, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=5)
        
        # Console Output
        tk.Label(self.root, text="Console Output:", padx=10).pack(anchor="w")
        self.console = tk.Text(self.root, bg="black", fg="lightgreen", height=20)
        self.console.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
    def edit_config(self):
        config_file = os.path.join(self.workspace_dir, "config", self.config_var.get())
        if not os.path.exists(config_file):
            messagebox.showerror("Error", f"Config file not found:\n{config_file}")
            return
            
        top = tk.Toplevel(self.root)
        top.title(f"Edit {self.config_var.get()}")
        top.geometry("800x600")
        
        text_area = tk.Text(top, wrap="none", font=("Courier", 10))
        text_area.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Load content
        try:
            with open(config_file, "r") as f:
                text_area.insert(tk.END, f.read())
        except Exception as e:
            messagebox.showerror("Error", str(e))
            top.destroy()
            return
            
        def save_file():
            try:
                with open(config_file, "w") as f:
                    f.write(text_area.get("1.0", tk.END))
                messagebox.showinfo("Success", "Configuration saved!")
                top.destroy()
            except Exception as e:
                messagebox.showerror("Error", str(e))
                
        btn_frame = tk.Frame(top)
        btn_frame.pack(fill=tk.X, pady=5)
        tk.Button(btn_frame, text="Save", bg="#2196F3", fg="white", command=save_file).pack(side=tk.RIGHT, padx=10)
        tk.Button(btn_frame, text="Cancel", command=top.destroy).pack(side=tk.RIGHT)

    def browse_bag(self):
        path = filedialog.askdirectory(title="Select ROS 2 Bag Directory")
        if path:
            self.bag_path_var.set(path)
            
    def log(self, message):
        self.console.insert(tk.END, message + "\n")
        self.console.see(tk.END)
        
    def stream_output(self, process):
        for line in iter(process.stdout.readline, ''):
            self.root.after(0, self.log, line.strip())
            
    def start_slam(self):
        bag_file = self.bag_path_var.get()
        if not bag_file:
            messagebox.showerror("Error", "Please select a ROS 2 bag file first.")
            return
            
        self.btn_run.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self.btn_save.config(state=tk.NORMAL)
        
        self.log("==================================================")
        self.log(f"=> Starting LIO-SAM with {self.config_var.get()}")
        self.log("==================================================")
        
        # Start LIO-SAM thread
        threading.Thread(target=self.run_slam_pipeline, args=(bag_file,), daemon=True).start()

    def run_slam_pipeline(self, bag_file):
        config_file = os.path.join(self.workspace_dir, "config", self.config_var.get())
        
        # Setup ROS environment
        setup_cmd = f"source /opt/ros/jazzy/setup.bash && source {self.workspace_dir}/install/setup.bash && "
        
        # 1. Launch LIO-SAM
        launch_cmd = setup_cmd + f"ros2 launch lio_sam run.launch.py params_file:={config_file}"
        self.lio_process = subprocess.Popen(launch_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, executable='/bin/bash', preexec_fn=os.setsid)
        
        # Stream LIO-SAM output
        threading.Thread(target=self.stream_output, args=(self.lio_process,), daemon=True).start()
        
        # Wait 5 seconds for initialization
        self.log("=> Waiting 5 seconds for LIO-SAM to initialize...")
        time.sleep(5)
        
        # 2. Play bag
        self.log(f"=> Playing dataset: {bag_file} with --clock")
        play_cmd = setup_cmd + f"ros2 bag play '{bag_file}' --clock"
        self.bag_process = subprocess.Popen(play_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, executable='/bin/bash', preexec_fn=os.setsid)
        
        self.stream_output(self.bag_process)
        self.log("=> Bag playback finished! Wait for RViz to stop moving, then click 'Save Map & Convert'.")

    def save_map(self):
        self.btn_save.config(state=tk.DISABLED)
        threading.Thread(target=self.run_save_and_convert, daemon=True).start()

    def run_save_and_convert(self):
        self.log("==================================================")
        self.log("=> Triggering Map Save via Service Call...")
        
        setup_cmd = f"source /opt/ros/jazzy/setup.bash && source {self.workspace_dir}/install/setup.bash && "
        
        # Call the service
        save_cmd = setup_cmd + f"ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \"{{resolution: 0.0, destination: '{self.map_dest}'}}\""
        
        try:
            output = subprocess.check_output(save_cmd, shell=True, text=True, executable='/bin/bash')
            self.log(output)
            self.log("=> Map saved to " + self.map_dest)
        except subprocess.CalledProcessError as e:
            self.log("=> ERROR saving map: " + str(e))
            self.btn_save.config(state=tk.NORMAL)
            return

        if self.convert_var.get():
            self.log("=> Converting GlobalMap.pcd to GlobalMap.ply...")
            convert_cmd = f"pcl_pcd2ply {self.map_dest}/GlobalMap.pcd {self.map_dest}/GlobalMap.ply"
            try:
                out = subprocess.check_output(convert_cmd, shell=True, text=True, stderr=subprocess.STDOUT)
                self.log(out)
                self.log("=> Successfully converted to GlobalMap.ply!")
            except subprocess.CalledProcessError as e:
                self.log("=> ERROR converting map: " + e.output)
                
        if self.convert_las_var.get():
            self.log("=> Converting GlobalMap.pcd to GlobalMap.las using PDAL...")
            las_cmd = f"pdal translate {self.map_dest}/GlobalMap.pcd {self.map_dest}/GlobalMap.las"
            try:
                out = subprocess.check_output(las_cmd, shell=True, text=True, stderr=subprocess.STDOUT)
                self.log(out)
                self.log("=> Successfully converted to GlobalMap.las!")
            except subprocess.CalledProcessError as e:
                if e.returncode == 127:
                    self.log("=> ERROR: PDAL not installed. Run: sudo apt install pdal")
                    messagebox.showerror("Missing Dependency", "To export to LAS, you must install PDAL.\nRun this in your terminal:\nsudo apt install pdal")
                else:
                    self.log("=> ERROR converting to LAS: " + e.output)
        
        self.log("=> Process Complete! You may stop the SLAM node.")

    def stop_all(self):
        self.log("=> Stopping processes...")
        if self.bag_process:
            try:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
            except Exception: pass
            self.bag_process = None
            
        if self.lio_process:
            try:
                os.killpg(os.getpgid(self.lio_process.pid), signal.SIGTERM)
            except Exception: pass
            self.lio_process = None
            
        self.btn_run.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.btn_save.config(state=tk.DISABLED)
        self.log("=> Stopped.")

if __name__ == "__main__":
    root = tk.Tk()
    app = LIOSAMApp(root)
    root.mainloop()
