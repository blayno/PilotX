# cnc_sender_full_maxspeed_grbl12h.py
# Optimized for GRBL 1.2h: keeps max speed streaming without triggering ALARM:11
# Jog tab merged into Sender tab; console smaller and under jog buttons
# Auto-level (grid probe) added — probes a grid, visualizes, and applies height corrections to loaded G-code.

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import serial # pyserial library is needed
import serial.tools.list_ports
import threading
import time
import queue
import re
from collections import deque
from math import floor
import csv
from matplotlib.figure import Figure # Matplotlib library is needed
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np # numpy library is needed
import json
from tkinter import simpledialog, messagebox
import os
from PIL import Image, ImageTk  # pillow library is needed

# ------------------------- Constants -------------------------
DEFAULT_SEND_RATE = 15.0  # lines/sec for simulation
READER_QUEUE_MAX = 1000   # serial response queue size
GRBL_BUFFER_MAX = 16      # GRBL 1.2h planner buffer (safe)

# ------------------------- CNC Sender App -------------------------
class CNCSenderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pilot CNC V1.2")
        self.root.geometry("1280x880")

        # Schedule the logo window to appear AFTER the GUI loads.
        self.root.after(300, self.show_logo_window)
        
        
        #Load Json File For Previous Settings
        self.config = {}
        self.load_settings()

        # Serial / Thread variables
        self.serial_connection = None
        self.is_connected = False
        self.serial_lock = threading.Lock()
        self.response_queue = queue.Queue(maxsize=READER_QUEUE_MAX)
        self.reader_thread = None

        self.send_manager_thread = None
        self.send_manager_stop = threading.Event()
        self.send_manager_pause = threading.Event()
        self.pending_lines = deque()  # lines sent but waiting for ok

        # G-code variables
        self.gcode_lines = []
        self.gcode_path = None
        self.total_lines = 0
        self.current_line_index = 0
        
        self.jog_distance = tk.DoubleVar(value=1.0)
        self.feedrate_entry_var = tk.StringVar(value="1000")
        self.send_rate = tk.DoubleVar(value=DEFAULT_SEND_RATE)
        self.simulate_mode = tk.BooleanVar(value=True)
        self.sim_speed = tk.DoubleVar(value=1.0)

        # Visualization
        self.vis_x, self.vis_y, self.vis_z = [], [], []

        # Status
        self.status_var = tk.StringVar(value="Idle")
        self._polling_paused = threading.Event()

        # Auto-level data structures
        self.al_xs = []
        self.al_ys = []
        self.al_heights = None  # 2D numpy array (ny x nx)
        self.al_ref_height = None  # reference height (used to compute corrections)
        self.corrected_gcode_lines = []
        
        # ---------------- Work coordinates (WPos) ----------------
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # ---------------- Machine coordinates (MPos) ----------------
        self.mpos_x = 0.0
        self.mpos_y = 0.0
        self.mpos_z = 0.0
        
        # Work coordinate offsets (WCO)
        self.wco_x = 0.0
        self.wco_y = 0.0
        self.wco_z = 0.0
        self.wco_a = 0.0
        self.wco_b = 0.0

        
        # Status polling
        self._last_status_request = 0
        self._status_poll_interval = 0.1  # seconds

        # Build UI
        self._build_ui()

        # Start background threads
        self.poll_thread = threading.Thread(target=self._position_poll_loop, daemon=True)
        self.poll_thread.start()
        self.response_handler_thread = threading.Thread(target=self._response_handler_loop, daemon=True)
        self.response_handler_thread.start()
        
            
# Logo in cmd and seperate window--------------------------------------------------


        # # Function to print ASCII version in console
        # def print_image_ascii(path, width=50):
            # if not os.path.exists(path):
                # print(f"Logo not found: {path}")
                # return

            # img = Image.open(path)
            # aspect_ratio = img.height / img.width
            # new_height = int(aspect_ratio * width * 0.55)
            # img = img.resize((width, new_height))
            # img = img.convert("L")  # grayscale

            # pixels = img.getdata()
            # chars = "@%#*+=-:. "  # 10 chars from dark to light
            # new_pixels = [chars[min(pixel * len(chars) // 256, len(chars)-1)] for pixel in pixels]
            # new_pixels = ''.join(new_pixels)

            # ascii_image = [new_pixels[i:i+width] for i in range(0, len(new_pixels), width)]
            # print("\n".join(ascii_image))



        # # Inside your main class or function
        # script_dir = os.path.dirname(os.path.abspath(__file__))
        # logo_path = os.path.join(script_dir, "images", "Pilot CNC Logo.png")

        # if os.path.exists(logo_path):
            # # --- GUI window ---
            # logo_image = Image.open(logo_path)
            # self.logo_photo = ImageTk.PhotoImage(logo_image)  # keep reference

            # logo_window = tk.Toplevel(self.root)
            # logo_window.title("PilotMill Logo")
            # logo_window.geometry(f"{logo_image.width}x{logo_image.height}")
            # logo_window.resizable(False, False)

            # logo_label = tk.Label(logo_window, image=self.logo_photo)
            # logo_label.pack()

            # # Center window
            # screen_width = logo_window.winfo_screenwidth()
            # screen_height = logo_window.winfo_screenheight()
            # x = (screen_width - logo_image.width) // 2
            # y = (screen_height - logo_image.height) // 2
            # logo_window.geometry(f"+{x}+{y}")

            # # --- Console ASCII preview ---
            # print_image_ascii(logo_path)

        # else:
            # print(f"Logo not found: {logo_path}")
            
# -------------------LOGO Function--------------------------------------
    def show_logo_window(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        logo_path = os.path.join(script_dir, "images", "Pilot CNC Logo.png")

        if os.path.exists(logo_path):
            logo_image = Image.open(logo_path)
            self.logo_photo = ImageTk.PhotoImage(logo_image)

            logo_window = tk.Toplevel(self.root)
            logo_window.title("PilotMill Logo")
            logo_window.geometry(f"{logo_image.width}x{logo_image.height}")
            logo_window.resizable(False, False)

            logo_label = tk.Label(logo_window, image=self.logo_photo)
            logo_label.pack()

            # Bring to front
            logo_window.lift()
            logo_window.attributes('-topmost', True)
            logo_window.after(500, lambda: logo_window.attributes('-topmost', False))

            # center
            screen_width = logo_window.winfo_screenwidth()
            screen_height = logo_window.winfo_screenheight()
            x = (screen_width - logo_image.width) // 2
            y = (screen_height - logo_image.height) // 2
            logo_window.geometry(f"+{x}+{y}")

            #print_image_ascii(logo_path)
            # Auto-close after 3 seconds
                    # === Fade-out after 3 seconds ===
            logo_window.after(5000, lambda: self._fade_out(logo_window))
            #logo_window.after(5000, logo_window.destroy)


        else:
            print(f"Logo not found: {logo_path}")
            
# logo fadeout -----------------------
    def _fade_out(self, window, alpha=1.0):
        if alpha > 0:
            alpha -= 0.05
            window.attributes("-alpha", alpha)
            window.after(50, lambda: self._fade_out(window, alpha))
        else:
            window.destroy()



        
                              
    # ------------------------- Build UI -------------------------
    def _build_ui(self):
        # Notebook: Sender tab + Auto-Level tab
        nb = ttk.Notebook(self.root)
        nb.pack(fill='both', expand=True, padx=6, pady=6)

        # --- Sender Tab ---
        sender_frame = ttk.Frame(nb, padding=8)
        nb.add(sender_frame, text="Sender")
        


        f = ttk.Frame(sender_frame, padding=4)
        f.pack(fill='both', expand=True)

        # --- Serial / G-code Controls ---
        ttk.Label(f, text="COM Port:").grid(row=0, column=0, sticky='e')
        self.port_cb = ttk.Combobox(f, values=self._list_serial_ports(), width=18)
        self.port_cb.grid(row=0, column=1,sticky='w', padx=4)
        if self.port_cb['values']:
            self.port_cb.current(0)

        ttk.Label(f, text="Baud:").grid(row=0, column=1, sticky='e')
        self.baud_cb = ttk.Combobox(f, values=["115200", "250000", "57600", "9600"], width=10)
        self.baud_cb.grid(row=0, column=2,sticky='w', padx=4)
        self.baud_cb.set("115200")

        ttk.Button(f, text="Refresh", command=self.refresh).grid(row=0, column=3,sticky='w', padx=6)
        ttk.Button(f, text="Connect", command=self.connect_serial).grid(row=0, column=4, padx=6)
        ttk.Button(f, text="Disconnect", command=self.disconnect_serial).grid(row=0, column=5, padx=6)
        ttk.Button(f, text="Unlock ($X)", command=self.unlock_machine).grid(row=0, column=6, padx=6)

        ttk.Button(f, text="Load G-code", command=self.load_gcode_file).grid(row=1, column=0, pady=8)
        ttk.Checkbutton(f, text="Simulation Mode", variable=self.simulate_mode).grid(row=1, column=1)
        ttk.Label(f, text="Sim Speed:").grid(row=1, column=2, sticky='e')
        ttk.Scale(f, from_=0.1, to=5.0, variable=self.sim_speed, orient='horizontal', length=160).grid(row=1, column=3)

        ttk.Button(f, text="Play", command=self.start_pipeline_send).grid(row=4, column=0)
        ttk.Button(f, text="Stop", command=self.stop_pipeline_send).grid(row=4, column=1)

        ttk.Button(f, text="Home (H)", command=lambda: self._send_line("$H")).grid(row=4, column=4, padx=6)
        ttk.Button(f, text="Feed Hold (!)", command=lambda: self._send_line("!")).grid(row=4, column=5, padx=6)
        ttk.Button(f, text="Cycle Start (~)", command=lambda: self._send_line("~")).grid(row=4, column=6, padx=6)

        ttk.Label(f, text="Status:").grid(row=4, column=2, sticky='e')
        ttk.Label(f, textvariable=self.status_var, foreground="blue").grid(row=4, column=3, sticky='w')

        self.progress = ttk.Progressbar(f, orient='horizontal', length=760, mode='determinate')
        self.progress.grid(row=5, column=0, columnspan=7, pady=8, sticky='ew')

        self.current_label = ttk.Label(f, text="Line: 0 / 0")
        self.current_label.grid(row=6, column=0, columnspan=3, sticky='w')
        self.eta_label = ttk.Label(f, text="ETA: -")
        self.eta_label.grid(row=6, column=3, columnspan=2)

        # --- Jog + Visualizer ---
        jog_vis_frame = ttk.Frame(f, padding=8)
        jog_vis_frame.grid(row=7, column=0, columnspan=7, sticky='nsew')
        jog_vis_frame.columnconfigure(0, weight=1)
        jog_vis_frame.columnconfigure(1, weight=1)

        # Jog controls on left
        left_frame = ttk.Frame(jog_vis_frame)
        left_frame.grid(row=0, column=0, sticky='nw')

        ttk.Label(left_frame, text="Feedrate (mm/min):").grid(row=0, column=0, sticky='e')
        self.feed_entry = ttk.Entry(left_frame, width=12, textvariable=self.feedrate_entry_var)
        self.feed_entry.grid(row=0, column=1, padx=4, sticky='w')

        ttk.Label(left_frame, text="Distance:").grid(row=0, column=2, sticky='e')
        self.distance_cb = ttk.Combobox(left_frame, values=[0.001, 0.1, 1, 10, 25, 50, 100], width=10, textvariable=self.jog_distance)
        self.distance_cb.grid(row=0, column=3, padx=4)
        try:
            self.distance_cb.current(2)
        except Exception:
            pass

        readout = ttk.LabelFrame(left_frame, text="Position (mm)")
        readout.grid(row=1, column=0, columnspan=4, pady=8, sticky='ew')
        
#------------------------------DRO and XYZ0 buttons-----------------------------------------
        # Work position
        self.readout_x = ttk.Label(readout, text=f"W: X: {self.pos_x:.3f}", width=18)
        self.readout_x.grid(row=0, column=0, padx=6)
        self.readout_y = ttk.Label(readout, text=f"W: Y: {self.pos_y:.3f}", width=18)
        self.readout_y.grid(row=0, column=1, padx=6)
        self.readout_z = ttk.Label(readout, text=f"W: Z: {self.pos_z:.3f}", width=18)
        self.readout_z.grid(row=0, column=2, padx=6)

        # Machine position
        self.readout_mx = ttk.Label(readout, text=f"M: X: {self.mpos_x:.3f}", width=18)
        self.readout_mx.grid(row=1, column=0, padx=6)
        self.readout_my = ttk.Label(readout, text=f"M: Y: {self.mpos_y:.3f}", width=18)
        self.readout_my.grid(row=1, column=1, padx=6)
        self.readout_mz = ttk.Label(readout, text=f"M: Z: {self.mpos_z:.3f}", width=18)
        self.readout_mz.grid(row=1, column=2, padx=6)

        ttk.Button(readout, text="Zero X", command=lambda: self.zero_axis('X')).grid(row=2, column=0, pady=6)
        ttk.Button(readout, text="Zero Y", command=lambda: self.zero_axis('Y')).grid(row=2, column=1)
        ttk.Button(readout, text="Zero Z", command=lambda: self.zero_axis('Z')).grid(row=2, column=2)
#-----------------------------------------------------------------------------------------------------------\

        btns = ttk.Frame(left_frame)
        btns.grid(row=2, column=0, columnspan=4, pady=8)

        ttk.Button(btns, text="↑", width=10, command=lambda: self.jog('Y+')).grid(row=0, column=1)
        ttk.Button(btns, text="←", width=10, command=lambda: self.jog('X-')).grid(row=1, column=0)
        ttk.Button(btns, text="→", width=10, command=lambda: self.jog('X+')).grid(row=1, column=2)
        ttk.Button(btns, text="↓", width=10, command=lambda: self.jog('Y-')).grid(row=2, column=1)
        ttk.Button(btns, text="Z+", width=10, command=lambda: self.jog('Z+')).grid(row=0, column=3)
        ttk.Button(btns, text="Z-", width=10, command=lambda: self.jog('Z-')).grid(row=2, column=3)

        ttk.Button(btns, text="G0 XY0", width=10, command=self.goxy0).grid(row=1, column=1)
        ttk.Button(btns, text="G0 Z0", width=10, command=self.goz0).grid(row=1, column=3)


        # ---------------------------------------------------------
        # Console + Manual Command (merged block)
        # ---------------------------------------------------------

        # Console
        console_frame = ttk.LabelFrame(left_frame, text="Console", padding=6)
        console_frame.grid(row=3, column=0, columnspan=4, pady=6, sticky='nsew')
        console_frame.columnconfigure(0, weight=1)

        self.console = scrolledtext.ScrolledText(console_frame, height=12, width=50)
        self.console.grid(row=0, column=0, columnspan=3, sticky='nsew')

        # Manual Command bar under console
        self.manual_cmd_var = tk.StringVar()

        ttk.Label(console_frame, text="Command:").grid(row=1, column=0, sticky='e', pady=(6,0))

        self.manual_entry = ttk.Entry(console_frame, width=40, textvariable=self.manual_cmd_var)
        self.manual_entry.grid(row=1, column=1, padx=4, pady=(6,0), sticky='ew')

        ttk.Button(console_frame, text="Send", command=lambda: self._send_manual_cmd())\
            .grid(row=1, column=2, padx=4, pady=(6,0))

               
        # Visualizer on right
        vis_frame = ttk.Frame(jog_vis_frame, padding=8)
        vis_frame.grid(row=0, column=1, sticky='ne', padx=10)

        # # Clear button moved ABOVE visualizer
        # ttk.Button(vis_frame, text="Clear Visualizer", command=self._clear_visualizer).pack(pady=(0, 6))

        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Toolpath Simulation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.canvas = FigureCanvasTkAgg(self.fig, master=vis_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        
                # logo on right of visualiser
        logo_frame = ttk.Frame(jog_vis_frame, padding=8)
        logo_frame.grid(row=0, column=2, sticky='ne', padx=10)
        
        from PIL import Image, ImageTk

        # Load the logo
        script_dir = os.path.dirname(os.path.abspath(__file__))
        logo_path = os.path.join(script_dir, "images", "Pilot CNC Logo2.png")
        logo_img_raw = Image.open(logo_path)
        logo_img_raw = logo_img_raw.resize((150, 150))  # optional resize
        logo_img = ImageTk.PhotoImage(logo_img_raw)

        # Create frame for logo
        logo_frame = ttk.Frame(jog_vis_frame, padding=8)
        logo_frame.grid(row=0, column=2, sticky='ne', padx=10)

        # Add the image inside the frame
        logo_label = ttk.Label(logo_frame, image=logo_img)
        logo_label.image = logo_img   # IMPORTANT: keep reference
        logo_label.pack()
        


        
        #----- Macro Tab ----
        macro_tab = ttk.Frame(nb, padding=8)
        nb.add(macro_tab, text='Macros')
        
        #----- Probe Tab ----
        probe_tab = ttk.Frame(nb, padding=8)
        nb.add(probe_tab, text='Probe')
        
        # --- Auto-Level Tab ---
        al_frame = ttk.Frame(nb, padding=8)
        nb.add(al_frame, text="Auto-Level")
  

        self._build_macro_ui(macro_tab)
        self._build_probe_ui(probe_tab)
        self._build_autolevel_ui(al_frame)                

                       
        
# --------------- Macro UI -------------------------------------------------
    # ---------------- Macro UI ----------------
    def _build_macro_ui(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill='both', expand=True, padx=10, pady=10)
                
        # ---- Add Macro Section ----
        add_frame = ttk.LabelFrame(frame, text="Add New Macro", padding=8)
        add_frame.pack(fill='x', pady=6)

        ttk.Label(add_frame, text="Macro Name:").grid(row=0, column=0, sticky='e')
        self.macro_name_entry = ttk.Entry(add_frame, width=25)
        self.macro_name_entry.grid(row=0, column=1, padx=5)

        ttk.Label(add_frame, text="G-code:").grid(row=1, column=0, sticky='ne')
        self.macro_text_entry = tk.Text(add_frame, width=50, height=6)
        self.macro_text_entry.grid(row=1, column=1, padx=5, pady=5)

        ttk.Button(add_frame, text="Add Macro", command=self._add_macro).grid(row=2, column=1, sticky='e', pady=5)

        # ---- Macro List Section ----
        list_frame = ttk.LabelFrame(frame, text="Saved Macros", padding=8)
        list_frame.pack(fill='both', expand=True)

        self.macro_listbox = tk.Listbox(list_frame, height=12)
        self.macro_listbox.pack(side='left', fill='both', expand=True)

        scrollbar = ttk.Scrollbar(list_frame, orient='vertical', command=self.macro_listbox.yview)
        scrollbar.pack(side='left', fill='y')
        self.macro_listbox.config(yscrollcommand=scrollbar.set)
        

        # ---- Buttons ----
        btn_frame = ttk.Frame(list_frame)
        btn_frame.pack(side='left', fill='y', padx=10)

        ttk.Button(btn_frame, text="Run Macro", width=16, command=self._run_selected_macro).pack(pady=5)
        ttk.Button(btn_frame, text="Delete Macro", width=16, command=self._delete_selected_macro).pack(pady=5)
        ttk.Button(btn_frame, text="Move Up", width=16, command=self._move_macro_up).pack(pady=5)
        ttk.Button(btn_frame, text="Move Down", width=16, command=self._move_macro_down).pack(pady=5)
        ttk.Button(btn_frame, text="Save Macros", width=16, command=self._save_macros).pack(pady=5)
        ttk.Button(btn_frame, text="Edit Macro", width=16, command=self._edit_selected_macro).pack(pady=5)

        #ttk.Button(btn_frame, text="Load Macros", width=16, command=self._load_macros).pack(pady=5)
        
        # Load macros from JSON
        self._load_macros()
        for name in self.macros.keys():
            self.macro_listbox.insert('end', name)



    # ------------------------- Auto-Level UI -------------------------
    def _build_autolevel_ui(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill='both', expand=True)

        # Grid parameters
        params = ttk.LabelFrame(frame, text="Grid Parameters", padding=6)
        params.grid(row=0, column=0, sticky='nw', padx=6, pady=6)

        ttk.Label(params, text="X start:").grid(row=0, column=0, sticky='e')
        # self.al_xstart = tk.DoubleVar(value=0.0)
        # ttk.Entry(params, textvariable=self.al_xstart, width=10).grid(row=0, column=1, padx=4)
        self.al_xstart = tk.DoubleVar(value=self.config.get("al_xstart", 0.0))
        self.al_xstart.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(params, textvariable=self.al_xstart, width=10).grid(row=0, column=1, padx=4)
              
        ttk.Label(params, text="X end:").grid(row=0, column=2, sticky='e')
        #self.al_xend = tk.DoubleVar(value=100.0)
        self.al_xend = tk.DoubleVar(value=self.config.get("al_xend", 0.0))
        self.al_xend.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(params, textvariable=self.al_xend, width=10).grid(row=0, column=3, padx=4)
        
        ttk.Label(params, text="X step:").grid(row=0, column=4, sticky='e')
        #self.al_xstep = tk.DoubleVar(value=20.0)
        self.al_xstep = tk.DoubleVar(value=self.config.get("al_xstep", 0.0))
        self.al_xstep.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(params, textvariable=self.al_xstep, width=10).grid(row=0, column=5, padx=4)

        ttk.Label(params, text="Y start:").grid(row=1, column=0, sticky='e')
        #self.al_ystart = tk.DoubleVar(value=0.0)
        self.al_ystart = tk.DoubleVar(value=self.config.get("al_ystart", 0.0))
        self.al_ystart.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(params, textvariable=self.al_ystart, width=10).grid(row=1, column=1, padx=4)
        
        

        ttk.Label(params, text="Y end:").grid(row=1, column=2, sticky='e')
        #self.al_yend = tk.DoubleVar(value=100.0)
        self.al_yend = tk.DoubleVar(value=self.config.get("al_yend", 0.0))
        self.al_yend.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(params, textvariable=self.al_yend, width=10).grid(row=1, column=3, padx=4)

        ttk.Label(params, text="Y step:").grid(row=1, column=4, sticky='e')
        #self.al_ystep = tk.DoubleVar(value=20.0)
        self.al_ystep = tk.DoubleVar(value=self.config.get("al_ystep", 0.0))
        self.al_ystep.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(params, textvariable=self.al_ystep, width=10).grid(row=1, column=5, padx=4)

        # Probe parameters
        probe_frame = ttk.LabelFrame(frame, text="Probe Settings", padding=6)
        probe_frame.grid(row=1, column=0, sticky='nw', padx=6, pady=6)

        ttk.Label(probe_frame, text="Probe command:").grid(row=0, column=0, sticky='e')
        self.al_probe_cmd = tk.StringVar(value="G38.2 Z-10 F100")
        ttk.Entry(probe_frame, textvariable=self.al_probe_cmd, width=28).grid(row=0, column=1, padx=4, pady=2, columnspan=3, sticky='w')

        ttk.Label(probe_frame, text="Safe Z (move before XY):").grid(row=1, column=0, sticky='e')
        #self.al_safe_z = tk.DoubleVar(value=5.0)
        self.al_safe_z = tk.DoubleVar(value=self.config.get("al_safe_z", 0.0))
        self.al_safe_z.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(probe_frame, textvariable=self.al_safe_z, width=10).grid(row=1, column=1, padx=4, sticky='w')

        ttk.Label(probe_frame, text="Pull-off (after probe):").grid(row=1, column=2, sticky='e')
        #self.al_pulloff = tk.DoubleVar(value=2.0)
        self.al_pulloff = tk.DoubleVar(value=self.config.get("al_pulloff", 0.0))
        self.al_pulloff.trace_add("write", lambda *args: self.save_ui_settings())
        ttk.Entry(probe_frame, textvariable=self.al_pulloff, width=10).grid(row=1, column=3, padx=4, sticky='w')

        # Auto-level actions
        actions = ttk.LabelFrame(frame, text=" Autolevel Actions", padding=6)
        actions.grid(row=2, column=0, sticky='nw', padx=6, pady=8)

        ttk.Button(actions, text="Start Probing", command=self.start_autolevel).grid(row=0, column=0, padx=6)                          
        ttk.Button(actions, text="Stop Probing", command=self.stop_autolevel).grid(row=0, column=1, padx=6)
        ttk.Button(actions, text="Visualize Map", command=self.visualize_al_map).grid(row=1, column=1, padx=6, pady=10)
        # ttk.Button(actions, text="Export CSV", command=self.export_al_csv).grid(row=1, column=2, padx=6, pady=10)
        ttk.Button(actions, text="Apply Correction to Loaded G-code", command=self.apply_height_map_to_gcode).grid(row=0, column=2, padx=6)
        ttk.Button(actions, text="Save Corrected G-code", command=self.save_corrected_gcode).grid(row=0, column=3, padx=6)
        
        ttk.Button(actions, text="AutoLevel Ready",command=self.AutolevelReady).grid(row=1, column=0, padx=10, pady=10)

# Map console / simple table 
        map_frame = ttk.LabelFrame(frame, text="Probe Log / Map", padding=6) 
        map_frame.grid(row=3, column=0, sticky='nsew', padx=6, pady=6) 
        map_frame.columnconfigure(0, weight=1) 
        self.al_console = scrolledtext.ScrolledText(map_frame, height=12, width=1) 
        self.al_console.grid(row=0, column=0, sticky='nsew')
        
        # Save UI Settings
        save_ui_settings2 = ttk.LabelFrame(frame, text="Save UI Settings", padding=6) 
        save_ui_settings2.grid(row=4, column=0, sticky='nw', padx=6, pady=6) 
        save_ui_settings2.columnconfigure(0, weight=1)
               
        ttk.Button(save_ui_settings2, text="Save UI Settings",command=self.update_ui_settings).grid(row=0, column=0, padx=10)
        
        tips_frame = ttk.Frame(frame)
        tips_frame.grid(row=6, column=0, rowspan=4, sticky='nw', padx=6)
        
        ttk.Label(tips_frame, text="Autolevel Ready : Move to rough center of probing area 5mm or less above the material\nRun Autolevel Ready to prime your height, then Start Probing \nMake sure probe is connected").grid(row=0, column=0, padx=10)


        # Map figure area (right)
        fig_frame = ttk.Frame(frame)
        fig_frame.grid(row=0, column=1, rowspan=4, sticky='nsew', padx=6)
        fig_frame.columnconfigure(0, weight=1)
        fig_frame.rowconfigure(0, weight=1)
        self.al_fig = Figure(figsize=(6,5), dpi=100)
        self.al_ax = self.al_fig.add_subplot(111, projection='3d')
        self.al_ax.set_title("Probe Map")
        self.al_canvas = FigureCanvasTkAgg(self.al_fig, master=fig_frame)
        self.al_canvas.get_tk_widget().pack(fill='both', expand=True)

        # Auto-level control state
        self._al_thread = None
        self._al_stop = threading.Event()
        self._al_lock = threading.Lock()
        
#---------------- Probe UI------------------------------------------------
    def _build_probe_ui(self, parent):
        # parent is the probe_tab passed in
        frame = ttk.Frame(parent)
        frame.pack(fill='both', expand=True)


        
        # ----- Z Probe Section -----
        z_frame = ttk.LabelFrame(frame, text="Z Probe", padding=8)
        z_frame.pack(fill='x', pady=5)
        
        ttk.Label(frame, text="Some text under the frame").pack(anchor='w')

        # ttk.Label(z_frame, text="Safe Z:").grid(row=0, column=0, sticky='e')
        # self.z_safe_entry = ttk.Entry(z_frame, width=8)
        # self.z_safe_entry.grid(row=0, column=1, padx=5)
        # #self.z_safe_entry.insert(0, "10.0")
        # self.z_safe_entry.insert(0, str(self.config["z_probe_Safe_Z"]))

        ttk.Label(z_frame, text="Probe Distance:").grid(row=0, column=0, sticky='e')
        self.z_dist_entry = ttk.Entry(z_frame, width=8)
        self.z_dist_entry.grid(row=0, column=1, padx=5)
        #self.z_dist_entry.insert(0, "-50.0")
        self.z_dist_entry.insert(0, str(self.config["z_probe_distance"]))
        

        ttk.Label(z_frame, text="Feed Rate:").grid(row=0, column=2, sticky='e')
        self.z_feed_entry = ttk.Entry(z_frame, width=8)
        self.z_feed_entry.grid(row=0, column=3, padx=5)
        #self.z_feed_entry.insert(0, "100.0")
        self.z_feed_entry.insert(0, str(self.config["z_probe_feed"]))
        

        # NEW — Touchplate Thickness
        ttk.Label(z_frame, text="Touchplate Thickness:").grid(row=1, column=0, sticky='e', pady=3)
        self.z_touch_entry = ttk.Entry(z_frame, width=8)
        self.z_touch_entry.grid(row=1, column=1, padx=5)
        #self.z_touch_entry.insert(0, "10.0")  # You can change default
        self.z_touch_entry.insert(0, str(self.config["z_touchplate_thickness"]))
        
                # NEW — Retract distance
        ttk.Label(z_frame, text="Retract:").grid(row=1, column=2, sticky='e', pady=3)
        self.z_retract_entry = ttk.Entry(z_frame, width=8)
        self.z_retract_entry.grid(row=1, column=3, padx=5)
        #self.z_touch_entry.insert(0, "10.0")  # You can change default
        self.z_retract_entry.insert(0, str(self.config["z_retract"]))
        
        

        ttk.Button(z_frame, text="Run Z Probe", 
                   command=self.z_probe_from_entries).grid(row=1, column=4, padx=10)



        # ----- X Center Probe Section -----
        x_frame = ttk.LabelFrame(frame, text="X Inside Center Probe", padding=8)
        x_frame.pack(fill='x', pady=5)

        ttk.Label(x_frame, text="Probe Distance:").grid(row=0, column=0, sticky='e')
        self.x_dist_entry = ttk.Entry(x_frame, width=8)
        self.x_dist_entry.grid(row=0, column=1, padx=5)
        #self.x_dist_entry.insert(0, "100.0")
        self.x_dist_entry.insert(0, str(self.config["x_prb_distance"]))
        

        ttk.Label(x_frame, text="Feed Rate:").grid(row=0, column=2, sticky='e')
        self.x_feed_entry = ttk.Entry(x_frame, width=8)
        self.x_feed_entry.grid(row=0, column=3, padx=5)
        #self.x_feed_entry.insert(0, "50.0")
        self.x_feed_entry.insert(0, str(self.config["x_prb_feed"]))
        

        ttk.Button(x_frame, text="Run X Center Probe", command=self.x_center_probe_from_entries).grid(row=0, column=4, padx=10)
        ttk.Label(x_frame, text="Retract is Probe Distance so measure your probing \nDont enter a random number or you might crash").grid(row=0, column=5, padx=10)

        # ----- Y Center Probe Section -----
        y_frame = ttk.LabelFrame(frame, text="Y Inside Center Probe", padding=8)
        y_frame.pack(fill='x', pady=5)

        ttk.Label(y_frame, text="Probe Distance:").grid(row=0, column=0, sticky='e')
        self.y_dist_entry = ttk.Entry(y_frame, width=8)
        self.y_dist_entry.grid(row=0, column=1, padx=5)
        #self.y_dist_entry.insert(0, "100.0")
        self.y_dist_entry.insert(0, str(self.config["y_prb_distance"]))

        ttk.Label(y_frame, text="Feed Rate:").grid(row=0, column=2, sticky='e')
        self.y_feed_entry = ttk.Entry(y_frame, width=8)
        self.y_feed_entry.grid(row=0, column=3, padx=5)
        #self.y_feed_entry.insert(0, "50.0")
        self.y_feed_entry.insert(0, str(self.config["y_prb_feed"]))        

        ttk.Button(y_frame, text="Run Y Center Probe", command=self.y_center_probe_from_entries).grid(row=0, column=4, padx=10)
        ttk.Label(y_frame, text="Retract is Probe Distance so measure your probing \nDont enter a random number or you might crash").grid(row=0, column=5, padx=10)
                  
        ss_frame = ttk.LabelFrame(frame, text="Save UI Settings", padding=8)
        ss_frame.pack(fill='x', pady=5)
        
        ttk.Button(ss_frame, text="Save UI Settings",
                  command=self.update_ui_settings).grid(row=0, column=0, padx=10)                

                  
    #------------------- Autolevel Ready Functions---------------------------------------
    def AutolevelReady(self):
                # Probe ready for Autolevel
        self._log(f"Probe Surface")
        self._send_line(f"G38.2 Z-5 F50")
        self._wait_for_ok(2.0)
        
        self._log(f"WCS = XYZ0")
        self._send_line(f"G10 L20 P1 X0 Y0 Z0")
        self._wait_for_ok(2.0)
                                            

    # ------------------------- Clear Visualizer Function -------------------------
    def _clear_visualizer(self):
        self.vis_x.clear()
        self.vis_y.clear()
        self.vis_z.clear()
        self.ax.cla()
        self.ax.set_title("Toolpath Simulation")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        try:
            self.canvas.draw_idle()
        except Exception:
            try:
                self.canvas.draw()
            except Exception:
                pass
#-------------- COM Port refrresh function---------------------------------------------------------------------
    def refresh(self):
        """Refresh COM port list and update the combobox."""
        # Get updated list of ports
        ports = self._list_serial_ports()

        # Update the combobox values
        self.port_cb['values'] = ports

        # Auto-select first port if available
        if ports:
            self.port_cb.current(0)
        else:
            self.port_cb.set("")  # Clear selection

        # Optional: logging
        if hasattr(self, "_log"):
            self._log(f"Refreshed COM ports: {ports}")

                
    #--------------------------- JSON settings Save Function -------------------------------------------------
    CONFIG_FILE = "settings.json"

    def load_settings(self):
        """Load settings from JSON file or create defaults."""
        if os.path.exists(self.CONFIG_FILE):
            try:
                with open(self.CONFIG_FILE, "r") as f:
                    self.config = json.load(f)
                print("Loaded settings.json")
            except Exception as e:
                print("Error loading settings.json:", e)
                self.config = {}
        else:
            print("settings.json not found, creating default settings.")
            self.config = {}

        # ---- Ensure Defaults Exist ----
        defaults = {
            #----- Sender Tab ------------
            "com_port": "COM1",
            "baud_rate": 115200,
            #------ Probe Tab ----------
            # "z_probe_Safe_Z": 10,
            "z_probe_distance": 50,
            "z_probe_feed": 50,
            "z_touchplate_thickness": 1,
            "x_prb_distance": 15,
            "x_prb_feed": 50,
            "y_prb_distance": 15,
            "y_prb_feed": 50,
            "z_retract": 5,
            #---------- Autolevel Tab-----------
            "al_xstart": -10,
            "al_xend": 10,
            "al_xstep": 20,
            "al_ystart": -10,
            "al_yend": 10,
            "al_ystep": 20,
            "al_safe_z": 5,
            "al_pulloff": 2,
            
            
            
        }

        # Merge any missing keys
        for key, val in defaults.items():
            if key not in self.config:
                self.config[key] = val

        # Save defaults if file didn't exist or was missing keys
        self.save_settings()


    def save_settings(self):
        """Save settings to JSON file."""
        try:
            with open(self.CONFIG_FILE, "w") as f:
                json.dump(self.config, f, indent=4)
            print("Saved settings.json")
        except Exception as e:
            print("Error saving settings.json:", e)
            
    # Save settings upon user input---------------
    def update_ui_settings(self):
        print("Updating GUI settings...")
        print("Before:", self.config)
        try:
            #---------- Probe Tab-----------
            # self.config["z_probe_Safe_Z"] = float(self.z_safe_entry.get())
            self.config["z_probe_distance"] = float(self.z_dist_entry.get())
            self.config["z_probe_feed"] = float(self.z_feed_entry.get())
            self.config["z_touchplate_thickness"] = float(self.z_touch_entry.get())
            self.config["x_prb_distance"] = float(self.x_dist_entry.get())
            self.config["x_prb_feed"] = float(self.x_feed_entry.get())
            self.config["y_prb_distance"] = float(self.y_dist_entry.get())
            self.config["y_prb_feed"] = float(self.y_feed_entry.get())
            self.config["z_retract"] = float(self.z_retract_entry.get())
            
            #-------- Autolevel Tab --------
            self.config["al_xstart"] = self.al_xstart.get()
            self.config["al_xend"] = self.al_xend.get()
            self.config["al_xstep"] = self.al_xstep.get()
            self.config["al_ystart"] = self.al_ystart.get()
            self.config["al_yend"] = self.al_yend.get()
            self.config["al_ystep"] = self.al_ystep.get()
            self.config["al_safe_z"] = self.al_safe_z.get()
            self.config["al_pulloff"] = self.al_pulloff.get()
            
            
            
            self.save_settings()
        except ValueError as e:
            print("Invalid input:", e)
        print("After:", self.config)
        



                
    #-----------------Manual Console Functions-----------------------------------------------------
    def _send_manual_cmd(self):
        cmd = self.manual_cmd_var.get().strip()
        if not cmd:
            return

        # Show in console
        try:
            self._append_console(f">> {cmd}")
        except:
            pass

        # Send to GRBL
        self._send_line(cmd)

        # Clear after sending
        self.manual_cmd_var.set("")
    
                
                

#------------------------Parse GRBL status lines safely Function--------------------------------------
    def _update_position_from_status(self, status_line):
        """
        Parse GRBL status line like:
        <Idle|MPos:5.000,10.000,0.000|WCO:1.000,2.000,0.000>
        """
        try:
            # Extract MPos
            mpos_match = re.search(r"MPos:([-.\d]+),([-.\d]+),([-.\d]+)", status_line)
            if mpos_match:
                self.mpos_x = float(mpos_match.group(1))
                self.mpos_y = float(mpos_match.group(2))
                self.mpos_z = float(mpos_match.group(3))
                # self.pos_z = self.Mpos_z

            # Extract WCO (work coordinate offset)
            wco_match = re.search(r"WCO:([-.\d]+),([-.\d]+),([-.\d]+)", status_line)
            if wco_match:
                self.wco_x = float(wco_match.group(1))
                self.wco_y = float(wco_match.group(2))
                self.wco_z = float(wco_match.group(3))

            # ---- Compute WPos manually (GRBL may not report WPos) ----
            if hasattr(self, "wco_x"):
                self.pos_x = self.mpos_x - self.wco_x
                self.pos_y = self.mpos_y - self.wco_y
                self.pos_z = self.mpos_z - self.wco_z
                

            # ---- Probe result (PRB) ----
            prb_match = re.search(r"PRB:([-.\d]+),([-.\d]+),([-.\d]+),[-.\d]+:(\d)", status_line)
            if prb_match:
                self.prb_x = float(prb_match.group(1))
                self.prb_y = float(prb_match.group(2))
                self.prb_z = float(prb_match.group(3))
                self.prb_ok = int(prb_match.group(4))


            # Update GUI labels
            self.root.after(0, self._update_position_labels)

            # print(
                # f"MPos: X={self.mpos_x:.3f}, Y={self.mpos_y:.3f}, Z={self.mpos_z:.3f} | "
                # f"WPos: X={self.pos_x:.3f}, Y={self.pos_y:.3f}, Z={self.pos_z:.3f}"
            # )

        except Exception as e:
            self._log(f"Error parsing position: {e}")
                  
                
    #-------------------------Macro Functions-----------------------------------------------------
    #---------------- Macro Data Persistence ----------------
    def _load_macros(self):
        self.macros_file = "macros.json"
        print("Macros Json Loaded...")
        self.macros = {}
        if os.path.exists(self.macros_file):
            try:
                with open(self.macros_file, "r") as f:
                    self.macros = json.load(f)
            except Exception:
                self.macros = {}

    def _save_macros(self):
        try:
            with open(self.macros_file, "w") as f:
                json.dump(self.macros, f, indent=4)
                print("Macros Json Saved...")
        except Exception as e:
            self._log(f"Failed to save macros: {e}")
            
    #---------------- Macro Management ----------------        
    def _add_macro(self):
        name = self.macro_name_entry.get().strip()
        gcode = self.macro_text_entry.get("1.0", "end").strip()
        if not name:
            self._log("Macro needs a name!")
            return
        if not gcode:
            self._log("Macro needs G-code!")
            return
        self.macros[name] = gcode
        self.macro_listbox.insert('end', name)
        self._save_macros()
        self.macro_name_entry.delete(0, 'end')
        self.macro_text_entry.delete("1.0", "end")
        self._log(f"Macro '{name}' added.")

    def _delete_selected_macro(self):
        sel = self.macro_listbox.curselection()
        if not sel:
            self._log("No macro selected.")
            return
        name = self.macro_listbox.get(sel[0])
        del self.macros[name]
        self.macro_listbox.delete(sel[0])
        self._save_macros()
        self._log(f"Macro '{name}' deleted.")

    def _move_macro_up(self):
        sel = self.macro_listbox.curselection()
        if not sel or sel[0] == 0:
            return
        i = sel[0]
        name = self.macro_listbox.get(i)
        self.macro_listbox.delete(i)
        self.macro_listbox.insert(i-1, name)
        self.macro_listbox.select_set(i-1)
        keys = list(self.macros.keys())
        idx = keys.index(name)
        keys[idx], keys[idx-1] = keys[idx-1], keys[idx]
        self.macros = {k: self.macros[k] for k in keys}
        self._save_macros()

    def _move_macro_down(self):
        sel = self.macro_listbox.curselection()
        if not sel:
            return
        i = sel[0]
        if i == self.macro_listbox.size() - 1:
            return
        name = self.macro_listbox.get(i)
        self.macro_listbox.delete(i)
        self.macro_listbox.insert(i+1, name)
        self.macro_listbox.select_set(i+1)
        keys = list(self.macros.keys())
        idx = keys.index(name)
        keys[idx], keys[idx+1] = keys[idx+1], keys[idx]
        self.macros = {k: self.macros[k] for k in keys}
        self._save_macros()
        
        
        
        #---------------- Run Macro (Threaded + Parameters) -------
    def _run_selected_macro(self):
        sel = self.macro_listbox.curselection()
        if not sel:
            self._log("No macro selected.")
            return
        name = self.macro_listbox.get(sel[0])
        gcode = self.macros.get(name, "")

        # Detect {param} placeholders
        params = re.findall(r"\{(.*?)\}", gcode)
        values = {}
        for p in params:
            val = simpledialog.askstring("Macro Parameter", f"Enter value for '{p}':")
            if val is None:
                self._log("Macro cancelled.")
                return
            values[p] = val

        # Replace parameters
        for k, v in values.items():
            gcode = gcode.replace(f"{{{k}}}", v)

        # Run macro in a separate thread
        def run_macro_thread():
            self._log(f"Running macro '{name}'...")
            for line in gcode.splitlines():
                line = line.strip()
                if line:
                    self._send_line(line)
                    self._wait_for_ok(1.0)
                    
            self._log(f"Macro '{name}' finished.")

        threading.Thread(target=run_macro_thread, daemon=True).start()
        
#--------------Edit macro functio-------------
    def _edit_selected_macro(self):
        selection = self.macro_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a macro to edit.")
            return

        name = self.macro_listbox.get(selection[0])
        original_gcode = self.macros.get(name, "")

        # --- Popup Window ---
        edit_win = tk.Toplevel(self.root)
        edit_win.title(f"Edit Macro - {name}")
        edit_win.geometry("450x350")
        edit_win.grab_set()   # make it modal

        ttk.Label(edit_win, text="Macro Name:").pack(anchor='w', padx=10, pady=5)
        name_entry = ttk.Entry(edit_win, width=40)
        name_entry.pack(padx=10)
        name_entry.insert(0, name)

        ttk.Label(edit_win, text="G-code:").pack(anchor='w', padx=10, pady=5)
        text_box = tk.Text(edit_win, width=55, height=12)
        text_box.pack(padx=10)
        text_box.insert('1.0', original_gcode)

        # --- Save logic ---
        def save_changes():
            new_name = name_entry.get().strip()
            new_gcode = text_box.get('1.0', 'end').strip()

            if not new_name:
                messagebox.showerror("Error", "Macro name cannot be empty.")
                return

            # If name changed, rename the key
            if new_name != name:
                self.macros.pop(name, None)
                # Update listbox
                self.macro_listbox.delete(selection[0])
                self.macro_listbox.insert(selection[0], new_name)

            self.macros[new_name] = new_gcode
            self._save_macros()
            edit_win.destroy()

        ttk.Button(edit_win, text="Save", command=save_changes).pack(side='right', padx=10, pady=10)
        ttk.Button(edit_win, text="Cancel", command=edit_win.destroy).pack(side='right')


               
    # ------------------------- Probe Functions -------------------------        
    def _do_z_probe(
            self,
            probe_dist,
            probe_feed,
            plate_thickness,
            retract,            
            verify_timeout=2.0
    ):

        # -----------------------------------------
        # Step 0: reset Z WCS to make retract easy
        # -----------------------------------------
        self._log("Step 0: Reset WCS Z=0")
        self._send_line("G90")              # Absolute
        self._wait_for_ok(0.1)
        self._send_line("G10 L20 P1 Z0")    # Zero Z offset in G54
        self._wait_for_ok(0.2)

        # -----------------------------------------
        # Step 1: First probe (fast) downward
        # -----------------------------------------
        self._log("Step 1: Probe")
        self._send_line("G91")  # relative
        self._wait_for_ok(0.1)

        # Probe downward (probe_dist is positive in UI, convert to negative)
        self._send_line(f"G38.2 Z-{abs(probe_dist):.4f} F{probe_feed:.1f}")
        self._wait_for_ok(6.0)
        self._send_line("G90")  # Absolute
        
        self._log("Step 2: Set active WCS Z0")
        self._send_line(f"G10 L20 P1 Z{plate_thickness:.4f}")
        self._wait_for_ok(0.2)

        
        # Retract from the touch plate
        self._log("Step 3: Retract")
        self._send_line("G91")  # relative
        self._wait_for_ok(0.1)
        self._send_line(f"G0 Z{retract:.4f}")  
        self._wait_for_ok(0.1)
        self._send_line("G90")  # Absolute
        self._wait_for_ok(0.1)
        

    def _do_x_center_probe(self, dist, feed, retract=2.0):
        
        self._log("Step 0: Reset WCS X=0")
        self._send_line("G90")              # Absolute
        self._wait_for_ok(0.1)
        self._send_line("G10 L20 P1 X0")    # Zero Z offset in G54
        self._wait_for_ok(0.2)
        """
        Perform X-centering probe using two touches:
        - Probe left
        - Retract
        - Probe right
        Then move to the true center.
        """
        # -----------------------------
        # 1. Go into relative mode
        # -----------------------------
        self._send_line("G91")

        # -----------------------------
        # 2. Probe left side
        # -----------------------------
        self._send_line(f"G38.2 X-{dist} F{feed}")
        self._wait_for_ok(5.0)
        x_left = self.pos_x  # machine reports actual contact X

        # # Retract forward a bit
        # self._send_line(f"G0 X{retract}")
        # self._wait_for_ok(2.0)

        # -----------------------------
        # 3. Sweep past center
        # -----------------------------
        self._send_line(f"G0 X{dist}")
        self._wait_for_ok(3.0)

        # -----------------------------
        # 4. Probe right side
        # -----------------------------
        self._send_line(f"G38.2 X{dist} F{feed}")  # move back left
        self._wait_for_ok(5.0)
        x_right = self.pos_x

        # Retract left
        self._send_line(f"G0 X-{retract}")
        self._wait_for_ok(2.0)

        # -----------------------------
        # 5. Compute true center
        # -----------------------------
        x_center = (x_left + x_right) / 2.0
        self._log(f"X center = {x_center:.4f}")

        # -----------------------------
        # 6. Move to center in ABSOLUTE
        # -----------------------------
        self._send_line("G90")  # back to absolute mode
        self._send_line(f"G0 X{x_center:.4f}")
        self._wait_for_ok(4.0)
        
        # Set WCS 0---------------------------------
        self._log(" Set WCS X=0")
        self._send_line("G90")              # Absolute
        self._wait_for_ok(0.1)
        self._send_line("G10 L20 P1 Y0")    # Zero Z offset in G54
        self._wait_for_ok(0.2)

        return x_center



    def _do_y_center_probe(self, dist, feed, retract=2.0):
        
        self._log("Reset WCS Y=0")
        self._send_line("G90")              # Absolute
        self._wait_for_ok(0.1)
        self._send_line("G10 L20 P1 Y0")    # Zero Z offset in G54
        self._wait_for_ok(0.2)
        """
        Probe Y front and back, then move to the exact center.
        """
        # -----------------------------
        # 1. Set to absolute first (safe)
        # -----------------------------
        self._send_line("G90")

        # -----------------------------
        # 2. Probe front (negative Y)
        # -----------------------------
        self._send_line("G91")  # relative mode
        self._send_line(f"G38.2 Y-{dist} F{feed}")
        self._wait_for_ok(5.0)
        y_front = self.pos_y

        # # Retract away from the probed surface
        # self._send_line(f"G0 Y{retract}")
        # self._wait_for_ok(2.0)

        # -----------------------------
        # 3. Move past center toward back
        # -----------------------------
        self._send_line(f"G0 Y{dist}")  
        self._wait_for_ok(3.0)

        # -----------------------------
        # 4. Probe back (positive Y)
        # -----------------------------
        self._send_line(f"G38.2 Y{dist} F{feed}")  # probe *toward* front again
        self._wait_for_ok(5.0)
        y_back = self.pos_y

        # Retract away from back surface
        self._send_line(f"G0 Y-{retract}")
        self._wait_for_ok(2.0)

        # -----------------------------
        # 5. Compute center
        # -----------------------------
        y_center = (y_front + y_back) / 2.0
        self._log(f"Y center = {y_center:.4f}")

        # -----------------------------
        # 6. Move to center in absolute mode
        # -----------------------------
        self._send_line("G90")
        self._send_line(f"G0 Y{y_center:.4f}")
        self._wait_for_ok(4.0)
        
        #Reset WCS to 0--------------
        
        self._log(" Set WCS Y=0")
        self._send_line("G90")              # Absolute
        self._wait_for_ok(0.1)
        self._send_line("G10 L20 P1 Y0")    # Zero Z offset in G54
        self._wait_for_ok(0.2)

        return y_center
        


                      
#----------------Macro Functions-----------------------------------------------------
    # Add macro
    def _add_macro(self):
        name = self.macro_name_entry.get().strip()
        gcode = self.macro_text_entry.get("1.0", "end").strip()

        if not name:
            self._log("Macro needs a name!")
            return
        if not gcode:
            self._log("Macro needs G-code!")
            return

        self.macros[name] = gcode
        self.macro_listbox.insert('end', name)

        self.macro_name_entry.delete(0, 'end')
        self.macro_text_entry.delete("1.0", "end")

        self._log(f"Macro '{name}' added.")


    def _run_selected_macro(self):
        sel = self.macro_listbox.curselection()
        if not sel:
            self._log("No macro selected.")
            return

        name = self.macro_listbox.get(sel[0])
        gcode = self.macros.get(name, "")

        self._log(f"Running macro '{name}'...")

        for line in gcode.splitlines():
            line = line.strip()
            if line:
                self._send_line(line)
                self._wait_for_ok(1.0)

        # Request a fresh GRBL status to update DRO
        self._send_line("?")
        self._wait_for_ok(0.2)



    # Delete macro
    def _delete_selected_macro(self):
        sel = self.macro_listbox.curselection()
        if not sel:
            self._log("No macro selected.")
            return

        name = self.macro_listbox.get(sel[0])

        del self.macros[name]
        self.macro_listbox.delete(sel[0])

        self._log(f"Macro '{name}' deleted.")

# ------------------------- Probe Functions From UI Entries -------------------------

    def z_probe_from_entries(self):
        """Read UI fields and start a threaded Z probe."""
        try:
            # safe_z = float(self.z_safe_entry.get())
            dist = float(self.z_dist_entry.get())
            feed = float(self.z_feed_entry.get())
            touch = float(self.z_touch_entry.get())
            retract = float(self.z_retract_entry.get())
            
        except ValueError:
            self._log("Invalid Z probe input.")
            return

        self._log("Starting Z probe...")
        threading.Thread(
            target=self._do_z_probe,
            args=(dist, feed, touch, retract),
            daemon=True
        ).start()


    def x_center_probe_from_entries(self):
        """Read UI fields and start X centering probe."""
        try:
            dist = float(self.x_dist_entry.get())
            feed = float(self.x_feed_entry.get())
        except ValueError:
            self._log("Invalid X probe input.")
            return

        self._log("Starting X center probe...")
        threading.Thread(
            target=self._do_x_center_probe,
            args=(dist, feed),
            daemon=True
        ).start()


    def y_center_probe_from_entries(self):
        """Read UI fields and start Y centering probe."""
        try:
            dist = float(self.y_dist_entry.get())
            feed = float(self.y_feed_entry.get())
        except ValueError:
            self._log("Invalid Y probe input.")
            return

        self._log("Starting Y center probe...")
        threading.Thread(
            target=self._do_y_center_probe,
            args=(dist, feed),
            daemon=True
        ).start()



    # -------------------------Serial Helper Functions -------------------------
    def _list_serial_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _log(self, text, widget=None):
        if widget is None:
            widget = self.console
        def append():
            try:
                widget.insert(tk.END, text + "\n")
                widget.see(tk.END)
            except Exception:
                pass
        try:
            self.root.after(0, append)
        except Exception:
            append()

    # ------------------------- Serial Functions -------------------------
    def connect_serial(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showwarning("Select port", "Choose a serial port first.")
            return
        try:
            baud = int(self.baud_cb.get())
        except Exception:
            baud = 115200
            self.baud_cb.set("115200")
        try:
            self.serial_connection = serial.Serial(port, baud, timeout=0.5)
            self.serial_connection.dtr = False
            self.serial_connection.rts = False
            time.sleep(0.05)
            self.serial_connection.dtr = True
            self.serial_connection.rts = True
            time.sleep(2.0)
            try:
                self.serial_connection.reset_input_buffer()
            except Exception:
                pass
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            return
        self.is_connected = True
        self._log(f"Connected to {port} @ {baud}")
        if self.reader_thread is None or not self.reader_thread.is_alive():
            self.reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self.reader_thread.start()

    def disconnect_serial(self):
        self.is_connected = False
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except Exception:
                pass
            self.serial_connection = None
        self._log("Serial disconnected")

    def unlock_machine(self):
        if self.is_connected:
            self._send_line("$X")
            self._log("Unlock command ($X) sent")
        else:
            messagebox.showwarning("Not connected", "Connect first.")

            
            
    #----------------DRO Thread-safe function to update labels-------------------------------------
    #----------------------Position Updates New-----------------------------------------
    def _update_position_labels(self):
        """Update DRO labels for both WPos and MPos in a thread-safe way."""
        def update():
            try:
                self.readout_x.config(text=f"W: X: {self.pos_x:.3f}")
                self.readout_y.config(text=f"W: Y: {self.pos_y:.3f}")
                self.readout_z.config(text=f"W: Z: {self.pos_z:.3f}")

                self.readout_mx.config(text=f"M: X: {self.mpos_x:.3f}")
                self.readout_my.config(text=f"M: Y: {self.mpos_y:.3f}")
                self.readout_mz.config(text=f"M: Z: {self.mpos_z:.3f}")
            except Exception:
                pass

        # Schedule the update on the main Tkinter thread
        try:
            if hasattr(self, "root"):  # replace with your Tk root variable
                self.root.after(0, update)
        except Exception:
            pass


    def zero_axis(self, axis):
        if axis == 'X':
            self.pos_x = 0.0
        elif axis == 'Y':
            self.pos_y = 0.0
        elif axis == 'Z':
            self.pos_z = 0.0
        self._update_position_labels()
        self._log(f"{axis} axis zeroed")
        if self.is_connected:
            self._send_line(f"G10 L20 P1 {axis}0")
            



    # ------------------------- G-code Send -------------------------
    def load_gcode_file(self):
        path = filedialog.askopenfilename(filetypes=[("G-code files","*.gcode *.nc *.tap"),("All files","*.*")])
        if not path:
            return
        with open(path, "r", encoding='utf-8', errors='ignore') as f:
            # remove empty lines and comments
            self.gcode_lines = [line.rstrip() for line in f if line.strip() and not line.strip().startswith(';')]

        # Inject G90 at the start
        self.gcode_lines.insert(0, "G90")

        self.gcode_path = path
        self.total_lines = len(self.gcode_lines)
        self.current_line_index = 0

        # >>> draw gcode on load <<<
        self._update_toolpath(gcode_lines=self.gcode_lines, redraw=True)
        self._log(f"Loaded {self.total_lines} lines from {path}")


    def _send_line(self, line):
        if self.simulate_mode.get():
            # Only log simulation commands that are not "?"
            if line != "?":
                self._log(f"[SIM] {line}")
            return

        if self.is_connected and self.serial_connection:
            try:
                with self.serial_lock:
                    self.serial_connection.write((line + "\n").encode('ascii', errors='ignore'))
                    self.serial_connection.flush()
                    # track pending lines for buffer management
                    self.pending_lines.append(line)
                    # Only log actual commands that are not "?"
                    if line != "?":
                        self._log(f">> {line}")
            except Exception as e:
                self._log(f"Serial write error: {e}")


    # ------------------------- Pipeline Send Optimized for GRBL 1.2h -------------------------
    def start_pipeline_send(self):
        if not self.gcode_lines:
            messagebox.showwarning("No G-code", "Load a G-code file first.")
            return

        # Draw the full toolpath on load if not already drawn
        self._update_toolpath(gcode_lines=self.gcode_lines, redraw=True)

        # Resume existing thread if paused
        if self.send_manager_thread and self.send_manager_thread.is_alive():
            self.send_manager_pause.clear()
            self.send_manager_stop.clear()
            self.status_var.set("Resuming...")
            return

        # Start a new sending thread
        self.send_manager_stop.clear()
        self.send_manager_pause.clear()
        self.send_manager_thread = threading.Thread(
            target=self._pipeline_send_loop_grbl12h, daemon=True
        )
        self.send_manager_thread.start()
        self.status_var.set("Running...")


    def pause_pipeline_send(self):
        self.send_manager_pause.set()
        self.status_var.set("Paused")


    def stop_pipeline_send(self):
        self.send_manager_stop.set()
        self.send_manager_pause.clear()
        self.status_var.set("Stopped")
        self.current_line_index = 0
        self.pending_lines.clear()
        try:
            self.progress['value'] = 0
            self.current_label.config(text="Line: 0 / 0")
        except Exception:
            pass


    def _pipeline_send_loop_grbl12h(self):
        update_interval = 25
        sim_yield = 0.002

        while self.current_line_index < self.total_lines:

            # --- Check STOP instantly ---
            if self.send_manager_stop.is_set():
                break

            # --- Handle Pause ---
            while self.send_manager_pause.is_set():
                if self.send_manager_stop.is_set():
                    break
                time.sleep(0.01)
            if self.send_manager_stop.is_set():
                break

            # --- GRBL Buffer wait (interruptible) ---
            while (
                not self.simulate_mode.get()
                and len(self.pending_lines) >= GRBL_BUFFER_MAX
            ):
                if self.send_manager_stop.is_set():
                    break
                time.sleep(0.001)
            if self.send_manager_stop.is_set():
                break

            # --- Get line ---
            line = self.gcode_lines[self.current_line_index]

            # --- Send to machine ---
            if not self.simulate_mode.get():
                self._send_line(line)

            # --- Update toolpath (throttled) ---
            redraw_now = (
                (self.current_line_index % update_interval == 0)
                or (self.current_line_index == self.total_lines - 1)
            )
            self._update_toolpath(gcode_line=line, redraw=redraw_now)

            self.current_line_index += 1

            # --- Safe GUI update ---
            if (
                self.current_line_index % update_interval == 0
                or self.current_line_index == self.total_lines
            ):
                try:
                    progress_val = (self.current_line_index / self.total_lines) * 100
                    self.progress.after(
                        0,
                        lambda v=progress_val: self.progress.config(value=v)
                    )
                    self.current_label.after(
                        0,
                        lambda i=self.current_line_index: self.current_label.config(
                            text=f"Line: {i} / {self.total_lines}"
                        )
                    )
                except Exception:
                    pass

            # --- Simulation mode sleep (interruptible) ---
            if self.simulate_mode.get():
                scaled = sim_yield / max(0.01, self.sim_speed.get())
                # break sleep into tiny slices so stop is instant
                end_time = time.time() + scaled
                while time.time() < end_time:
                    if self.send_manager_stop.is_set():
                        break
                    time.sleep(0.001)
                if self.send_manager_stop.is_set():
                    break

        # Finish
        self.status_var.set("Idle")



    # ------------------------- Visualization -------------------------
    def _update_toolpath(self, gcode_line=None, gcode_lines=None, redraw=True):
        """
        Draws the full toolpath from G-code lines and moves a yellow cone for the current position.
        - gcode_lines: list of lines to draw the complete path (on load)
        - gcode_line: single line to move the cone along the path
        """
        import re
        import numpy as np

        # ----------------- Initialize axes if not done -----------------
        if not hasattr(self, 'ax') or self.ax is None:
            from mpl_toolkits.mplot3d import Axes3D  # needed for 3D
            import matplotlib.pyplot as plt
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
            # If using Tkinter:
            try:
                from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
                self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
                self.canvas.draw()
            except Exception:
                pass

        # ----------------- Draw full toolpath on load -----------------
        if gcode_lines:
            self.vis_x = []
            self.vis_y = []
            self.vis_z = []

            sim_x, sim_y, sim_z = 0, 0, 0

            for line in gcode_lines:
                # Match X, Y, Z with optional spaces
                coords = re.findall(r"[XYZ]\s*-?\d+\.?\d*", line, flags=re.IGNORECASE)
                for c in coords:
                    try:
                        axis = c[0].upper()
                        val = float(c[1:].strip())
                        if axis == 'X':
                            sim_x = val
                        elif axis == 'Y':
                            sim_y = val
                        elif axis == 'Z':
                            sim_z = val
                    except Exception:
                        pass

                self.vis_x.append(sim_x)
                self.vis_y.append(sim_y)
                self.vis_z.append(sim_z)

            if redraw:
                try:
                    self.ax.cla()
                    self.ax.plot(self.vis_x, self.vis_y, self.vis_z, color='blue', linewidth=2)

                    # Set axes limits with padding
                    padding = 5
                    if self.vis_x and self.vis_y and self.vis_z:
                        self.ax.set_xlim(min(self.vis_x)-padding, max(self.vis_x)+padding)
                        self.ax.set_ylim(min(self.vis_y)-padding, max(self.vis_y)+padding)
                        self.ax.set_zlim(min(self.vis_z)-padding, max(self.vis_z)+padding)

                        # Set aspect ratio proportional to ranges
                        dx = max(self.vis_x)-min(self.vis_x)+1
                        dy = max(self.vis_y)-min(self.vis_y)+1
                        dz = max(self.vis_z)-min(self.vis_z)+1
                        self.ax.set_box_aspect([dx, dy, dz])

                    self.ax.set_xlabel("X")
                    self.ax.set_ylabel("Y")
                    self.ax.set_zlabel("Z")
                    self.ax.set_title("Toolpath Simulation")

                    # Draw canvas
                    try:
                        self.canvas.draw_idle()
                    except Exception:
                        self.canvas.draw()
                except Exception as e:
                    self._log(f"Visualization error: {e}")

        # ----------------- Move yellow cone for single G-code line -----------------
        if gcode_line:
            coords = re.findall(r"[XYZ]\s*-?\d+\.?\d*", gcode_line, flags=re.IGNORECASE)
            x, y, z = getattr(self, "pos_x", 0), getattr(self, "pos_y", 0), getattr(self, "pos_z", 0)

            for c in coords:
                try:
                    axis = c[0].upper()
                    val = float(c[1:].strip())
                    if axis == 'X':
                        x = val
                        self.pos_x = val
                    elif axis == 'Y':
                        y = val
                        self.pos_y = val
                    elif axis == 'Z':
                        z = val
                        self.pos_z = val
                except Exception:
                    pass

            if redraw:
                try:
                    tx, ty, tz = x, y, z
                    cone_height = 5
                    cone_radius = 2
                    segments = 20

                    theta = np.linspace(0, 2*np.pi, segments)
                    r = np.linspace(0, cone_radius, 2)
                    T, R = np.meshgrid(theta, r)

                    Xc = tx + R * np.cos(T)
                    Yc = ty + R * np.sin(T)
                    Zc = tz + (cone_height * (R / cone_radius))

                    # Remove previous cone surfaces safely
                    for c in list(self.ax.collections):
                        if hasattr(c, "_is_cone") and c._is_cone:
                            c.remove()

                    # Draw new cone
                    cone_surf = self.ax.plot_surface(
                        Xc, Yc, Zc,
                        color='yellow',
                        edgecolor='none',
                        shade=True,
                        alpha=0.8
                    )
                    cone_surf._is_cone = True

                    try:
                        self.canvas.draw_idle()
                    except Exception:
                        self.canvas.draw()
                except Exception as e:
                    self._log(f"Cone draw error: {e}")

        # ----------------- Update GUI labels -----------------
        self._update_position_labels()




    # ------------------------- Serial Reader Loop -------------------------
    # def _log_safe(self, msg):
        # """Thread-safe console logger that ignores GRBL noise like ok, ?, MPos, WPos."""
        # if not msg:
            # return

        # line = str(msg).strip()

        # # Filter out unwanted GRBL echo/status lines
        # if line.lower() == "ok" or line == "?" or line.startswith("MPos:") or line.startswith("WPos:"):
            # return  # silently ignore

        # # Thread-safe print or GUI log
        # try:
            # print(line)
            # # If using a Tkinter Text or Listbox for logs:
            # # self.log_text.insert('end', line + "\n")
            # # self.log_text.see('end')
        # except Exception:
            # pass


    def _serial_reader_loop(self):
        """GRBL serial reader: updates DRO, manages queue, and keeps console clean."""
        buffer = ""
        while True:
            if self.is_connected and self.serial_connection:
                try:
                    raw = self.serial_connection.readline()
                    if not raw:
                        time.sleep(0.001)
                        continue

                    line = raw.decode('ascii', errors='ignore').strip()
                    if not line:
                        continue

                    # ------------------------------------------------------
                    # 🔥 IGNORE ALARM 11 (GRBL Mega-5X hard-limit false trigger)
                    # ------------------------------------------------------
                    if line.startswith("ALARM:11"):
                        self._log_safe("Ignoring ALARM:11 (auto-reset)")
                        self._send_line("\x18")   # CTRL-X soft reset
                        time.sleep(0.05)
                        self._send_line("$X")     # Unlock GRBL
                        continue  # DO NOT stop or add to any queue

                    # ------------------------------------------------------
                    # DRO / Position updates
                    # ------------------------------------------------------
                    if line.startswith("<") and "MPos:" in line:
                        self._update_position_from_status(line)

                        if "WCO:" in line:
                            try:
                                wco = line.split("WCO:")[1].split("|")[0]
                                wx, wy, wz = map(float, wco.split(","))
                                self.wco_x = wx
                                self.wco_y = wy
                                self.wco_z = wz
                            except:
                                pass
                                
                        # hide idle/Mposs lines from console-----------
                        self.pos_x = self.mpos_x - self.wco_x
                        self.pos_y = self.mpos_y - self.wco_y
                        self.pos_z = self.mpos_z - self.wco_z
                        continue

                    # ------------------------------------------------------
                    # Handle OK -> dequeue next G-code line
                    # ------------------------------------------------------
                    if line.lower() == "ok" and self.pending_lines:
                        try:
                            self.pending_lines.popleft()
                        except Exception:
                            pass
                        continue

                    # ------------------------------------------------------
                    # Add GRBL response to the queue
                    # ------------------------------------------------------
                    try:
                        if self.response_queue.full():
                            try:
                                self.response_queue.get_nowait()
                            except Exception:
                                pass
                        self.response_queue.put_nowait(line)
                    except Exception:
                        pass

                    # ------------------------------------------------------
                    # Log line
                    # ------------------------------------------------------
                    self._log_safe(line)

                except Exception:
                    time.sleep(0.01)

            else:
                time.sleep(0.1)





    # ------------------------- Response Handler -------------------------
    def _response_handler_loop(self):
        while True:
            try:
                if not self.response_queue.empty():
                    try:
                        line = self.response_queue.get(timeout=0.1)
                        self._log(f"<< {line}")
                    except queue.Empty:
                        pass
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    # ------------------------- Position Poll Loop -------------------------
    def _position_poll_loop(self):
        while True:
            if self.is_connected and not self._polling_paused.is_set():
                # Send "?" without logging to console (handled in _send_line)
                # self._log("?")
                self._send_line("?")
                
            time.sleep(0.2)



    # ------------------------- Jog Commands -------------------------
    def jog(self, direction):
        distance = self.jog_distance.get()
        feed = self.feedrate_entry_var.get()
        try:
            feed = float(feed)
        except Exception:
            feed = 1000
            self.feedrate_entry_var.set("1000")

        cmd = ""
        if direction == 'X+':
            self.pos_x += distance
            cmd = f"G91 G0 X{distance} F{feed}"
        elif direction == 'X-':
            self.pos_x -= distance
            cmd = f"G91 G0 X-{distance} F{feed}"
        elif direction == 'Y+':
            self.pos_y += distance
            cmd = f"G91 G0 Y{distance} F{feed}"
        elif direction == 'Y-':
            self.pos_y -= distance
            cmd = f"G91 G0 Y-{distance} F{feed}"
        elif direction == 'Z+':
            self.pos_z += distance
            cmd = f"G91 G0 Z{distance} F{feed}"
        elif direction == 'Z-':
            self.pos_z -= distance
            cmd = f"G91 G0 Z-{distance} F{feed}"

        self._update_position_labels
        self._log(f"Jog: {direction} ({distance} mm)")
        if self.is_connected:
            self._send_line(cmd)
            
            
    #------------------------- G0XY0 G0Z0 Functions---------------------------------------
    def goxy0(self):
        self._log("G0 X0 Y0...")

        self._send_line("G90")        # Absolute mode
        self._send_line("G0 X0 Y0")
        self._wait_for_ok(1)          # Wait for line acknowledgment

        self._log("Command sent: Gone to X0 Y0")
        # DRO will update automatically when GRBL sends status report


    def goz0(self):
        self._log("G0 Z0...")

        self._send_line("G90")        # Absolute mode
        self._send_line("G1 Z0 F100") # Controlled feed
        self._wait_for_ok(1)          # Wait for acknowledgment

        self._log("Command sent: Gone to Z0")
        # DRO updates will come from serial reader automatically



    # ------------------------- Auto-Leveling Core -------------------------
    def start_autolevel(self):
        if self._al_thread and self._al_thread.is_alive():
            messagebox.showinfo("Auto-Level", "Probing already running.")
            return
        if not self.is_connected and not self.simulate_mode.get():
            if messagebox.askyesno("Not connected", "Serial not connected. Run in simulation mode instead?"):
                self.simulate_mode.set(True)
            else:
                return

        xs = self._frange(self.al_xstart.get(), self.al_xend.get(), self.al_xstep.get())
        ys = self._frange(self.al_ystart.get(), self.al_yend.get(), self.al_ystep.get())
        if len(xs) == 0 or len(ys) == 0:
            messagebox.showerror("Invalid grid", "Grid parameters produce zero points.")
            return

        with self._al_lock:
            self.al_xs = xs
            self.al_ys = ys
            self.al_heights = np.full((len(ys), len(xs)), np.nan, dtype=float)
            self.al_ref_height = None

        self._al_stop.clear()
        self._al_thread = threading.Thread(target=self._autolevel_thread, daemon=True)
        self._al_thread.start()
        self._log("Auto-level: started probing", widget=self.al_console)


    def stop_autolevel(self):
        self._al_stop.set()
        self._log("Auto-level: stop requested", widget=self.al_console)


    def _frange(self, start, end, step):
        if step == 0:
            return []
        pts = []
        if step > 0:
            v = start
            while v <= end + 1e-9:
                pts.append(round(v, 6))
                v += step
        else:
            v = start
            while v >= end - 1e-9:
                pts.append(round(v, 6))
                v += step
        return pts


    def _autolevel_thread(self):
        xs = self.al_xs[:]
        ys = self.al_ys[:]
        probe_cmd_template = self.al_probe_cmd.get().strip()
        safe_z = float(self.al_safe_z.get())
        pulloff = float(self.al_pulloff.get())
        total_pts = len(xs) * len(ys)
        idx = 0

        # Precompute center offsets for simulation
        x_center = 0.5 * (xs[0] + xs[-1]) if xs else 0
        y_center = 0.5 * (ys[0] + ys[-1]) if ys else 0

        for iy, y in enumerate(ys):
            row_xs = xs[:] if (iy % 2 == 0) else list(reversed(xs))
            for ix, x in enumerate(row_xs):
                if self._al_stop.is_set():
                    self._log("Auto-level: stopped by user", widget=self.al_console)
                    return

                idx += 1
                self._log(f"Probing point {idx}/{total_pts}: X={x}, Y={y}", widget=self.al_console)

                # Move to safe Z and XY
                self._send_line(f"G90 G0 Z{safe_z}")
                self._wait_for_ok(timeout=0.1)
                self._send_line(f"G90 G0 X{x} Y{y}")
                self._wait_for_ok(timeout=0.1)

                # Probe with retry and alarm handling
                measured = None
                for attempt in range(3):
                    if self.simulate_mode.get():
                        # --- SIMULATED BED: gentle curved surface ---
                        dx = x - x_center
                        dy = y - y_center
                        sim_z = 0.002 * dx + 0.0015 * dy           # slight tilt
                        sim_z += 0.003 * np.sin(dx / max(1.0, xs[-1]-xs[0]))  # gentle bumps
                        sim_z += 0.003 * np.cos(dy / max(1.0, ys[-1]-ys[0]))
                        measured = float(sim_z)
                        time.sleep(0.08)
                        self._log(f"[SIM PROBE] Z={measured:.4f}", widget=self.al_console)
                        break
                    else:
                        self._drain_response_queue()
                        self._send_line(probe_cmd_template)
                        start_time = time.time()
                        while time.time() - start_time < 6.0:
                            try:
                                line = self.response_queue.get(timeout=0.1)
                            except queue.Empty:
                                continue
                            line_lower = line.strip().lower()
                            m = re.search(r"prb[:=]\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)", line_lower)
                            if m:
                                measured = float(m.group(3))  # <-- use Z value (3rd number)

                                self._log(f"Measured Z Mpos = {measured:.6f} (X={m.group(1)}, Y={m.group(2)})", widget=self.al_console)
                               
                                #---------------- Display Wpos Autolevel---------------------------
                                mx = float(m.group(1))        # MPos X
                                my = float(m.group(2))        # MPos Y
                                mz = measured                 # MPos Z from PRB

                                # Convert to WPos
                                wx = mx - self.wco_x
                                wy = my - self.wco_y
                                wz = mz - self.wco_z

                                self._log(
                                    f"Measured Z Wpos = {wz:.6f} (WPos X={wx:.3f}, Y={wy:.3f})",
                                    widget=self.al_console
)                               #----------------------------------------------------------------------
                                self._send_line(f"G91 G0 Z{pulloff}")
                                self._wait_for_ok(timeout=0.1)
                                self._send_line("G90")
                                self._wait_for_ok(timeout=0.1)
                                break
                            if "alarm" in line_lower or "error" in line_lower:
                                self._log(f"Probe triggered alarm: {line}", widget=self.al_console)
                                self._send_line(f"G91 G0 Z{pulloff}")
                                self._wait_for_ok(timeout=0.1)
                                self._send_line("G90")
                                self._wait_for_ok(timeout=0.1)
                                measured = float('nan')
                                break
                        if measured is not None:
                            break
                        time.sleep(0.2)
                if measured is None:
                    measured = float('nan')


                # Map measured value to correct column for snake pattern
                actual_ix = ix if (iy % 2 == 0) else (len(xs) - 1 - ix)
                actual_iy = iy

                with self._al_lock:
                    # Convert probed Z from MPos → WPos
                    measured_w = measured - self.wco_z

                    self.al_heights[actual_iy, actual_ix] = measured_w



                try:
                    self._update_al_partial_plot()
                except Exception:
                    pass

            time.sleep(0.05)

        self._log("Auto-level: probing complete", widget=self.al_console)




    def _wait_for_ok(self, timeout=5.0):
        """
        Waits for GRBL 'ok', but is fully interruptible.
        Returns:
            True  -> ok received
            False -> stopped, error, alarm, or timeout
        """

        deadline = time.time() + timeout

        while time.time() < deadline:

            # --- INSTANT STOP ---
            if self.send_manager_stop.is_set():
                return False

            try:
                # Very short timeout so stop interruption is instant
                line = self.response_queue.get(timeout=0.02)

            except queue.Empty:
                continue

            if not line:
                continue

            line = line.strip().lower()

            if line == "ok":
                return True

            if "error" in line or "alarm" in line:
                self._log(f"GRBL reported: {line}", widget=self.al_console)
                return False

        # Timeout hit
        return False



    def _drain_response_queue(self):
        try:
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except Exception:
                    break
        except Exception:
            pass


    # ------------------------- Auto-Level Visualization / Export -------------------------
    def _update_al_partial_plot(self):
        with self._al_lock:
            xs = self.al_xs
            ys = self.al_ys
            hs = self.al_heights.copy()
        self.al_ax.cla()
        self.al_ax.set_title("Probe Map")
        self.al_ax.set_xlabel("X")
        self.al_ax.set_ylabel("Y")
        self.al_ax.set_zlabel("Z")

        # --- FIXED snake-pattern visualization ---
        xs_list, ys_list, zs_list = [], [], []
        for iy, y in enumerate(ys):
            row_xs = xs[:] if (iy % 2 == 0) else list(reversed(xs))
            for ix, x in enumerate(row_xs):
                actual_ix = ix if (iy % 2 == 0) else (len(xs) - 1 - ix)
                h = hs[iy, actual_ix]
                if not np.isnan(h):
                    xs_list.append(x)
                    ys_list.append(y)
                    zs_list.append(h)

        if xs_list:
            self.al_ax.scatter(xs_list, ys_list, zs_list)

        try:
            self.al_canvas.draw_idle()
        except Exception:
            try:
                self.al_canvas.draw()
            except Exception:
                pass


    def visualize_al_map(self):
        with self._al_lock:
            xs = self.al_xs[:]
            ys = self.al_ys[:]
            hs = None if self.al_heights is None else self.al_heights.copy()
        if not xs or not ys or hs is None:
            messagebox.showwarning("No map", "No probe data to visualize.")
            return

        X, Y = np.meshgrid(xs, ys)
        Z = hs.copy()

        # --- Fill NaNs by neighbor averaging ---
        nan_mask = np.isnan(Z)
        if np.any(nan_mask):
            for iy in range(Z.shape[0]):
                for ix in range(Z.shape[1]):
                    if np.isnan(Z[iy, ix]):
                        neigh = []
                        for dy in (-1, 1):
                            y2 = iy + dy
                            if 0 <= y2 < Z.shape[0]:
                                v = Z[y2, ix]
                                if not np.isnan(v):
                                    neigh.append(v)
                        for dx in (-1, 1):
                            x2 = ix + dx
                            if 0 <= x2 < Z.shape[1]:
                                v = Z[iy, x2]
                                if not np.isnan(v):
                                    neigh.append(v)
                        Z[iy, ix] = np.mean(neigh) if neigh else 0.0

        # --- FIXED: un-snake the odd rows before plotting ---
        Z_plot = np.zeros_like(Z)
        for iy in range(len(ys)):
            if iy % 2 == 0:
                Z_plot[iy, :] = Z[iy, :]
            else:
                Z_plot[iy, :] = Z[iy, ::-1]

        self.al_ax.cla()
        self.al_ax.plot_surface(X, Y, Z_plot, cmap='viridis', edgecolor='none', rstride=1, cstride=1)
        self.al_ax.scatter(X.flatten(), Y.flatten(), Z_plot.flatten(), s=8)
        self.al_ax.set_xlabel("X")
        self.al_ax.set_ylabel("Y")
        self.al_ax.set_zlabel("Z")
        self.al_ax.set_title("Probe Map Surface")

        try:
            self.al_canvas.draw_idle()
        except Exception:
            try:
                self.al_canvas.draw()
            except Exception:
                pass


    def export_al_csv(self):
        with self._al_lock:
            xs = self.al_xs[:]
            ys = self.al_ys[:]
            hs = None if self.al_heights is None else self.al_heights.copy()
        if not xs or not ys or hs is None:
            messagebox.showwarning("No map", "No probe data to export.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv"),("All files","*.*")])
        if not path:
            return
        try:
            with open(path, "w", newline='') as csvf:
                writer = csv.writer(csvf)
                header = ["Y\\X"] + xs
                writer.writerow(header)
                for iy, y in enumerate(ys):
                    row = [y] + [("{:.6f}".format(hs[iy, ix]) if not np.isnan(hs[iy, ix]) else "") for ix in range(len(xs))]
                    writer.writerow(row)
            messagebox.showinfo("Exported", f"Probe map saved to {path}")
        except Exception as e:
            messagebox.showerror("Export failed", str(e))



    def apply_height_map_to_gcode(self):
        if not self.gcode_lines:
            messagebox.showwarning("No G-code", "Load a G-code file first.")
            return
        with self._al_lock:
            if self.al_heights is None or len(self.al_xs) == 0 or len(self.al_ys) == 0:
                messagebox.showwarning("No map", "No probe map available. Run probing first.")
                return
            xs = self.al_xs[:]
            ys = self.al_ys[:]
            hs = self.al_heights.copy()
            ref = self.al_ref_height if self.al_ref_height is not None else 0.0

        corrected = []
        cur_x = None
        cur_y = None
        absolute_mode = True
        for line in self.gcode_lines:
            ln = line.strip()
            if re.search(r"\bG90\b", ln):
                absolute_mode = True
            if re.search(r"\bG91\b", ln):
                absolute_mode = False
            x_m = re.search(r"X(-?\d+\.?\d*)", ln, flags=re.IGNORECASE)
            y_m = re.search(r"Y(-?\d+\.?\d*)", ln, flags=re.IGNORECASE)
            z_m = re.search(r"Z(-?\d+\.?\d*)", ln, flags=re.IGNORECASE)
            if x_m:
                cur_x = float(x_m.group(1))
            if y_m:
                cur_y = float(y_m.group(1))
            if z_m:
                z_val = float(z_m.group(1))
                lookup_x = cur_x
                lookup_y = cur_y
                if lookup_x is None or lookup_y is None:
                    corrected.append(line)
                    continue
                surf_h = self._get_height_at(lookup_x, lookup_y, xs, ys, hs)
                if surf_h is None or np.isnan(surf_h):
                    corrected.append(line)
                    continue
                delta = surf_h - ref
                new_z = z_val + delta
                new_line = re.sub(r"(Z)-?\d+\.?\d*", lambda m: f"Z{new_z:.6f}", line, flags=re.IGNORECASE)
                corrected.append(new_line)
            else:
                corrected.append(line)
        self.corrected_gcode_lines = corrected
        messagebox.showinfo("Applied", "Height map corrections applied to loaded G-code (in-memory). Use 'Save Corrected G-code' to write to file.")
        self._log("Auto-level: corrections applied (in-memory)")


    def save_corrected_gcode(self):
        if not self.corrected_gcode_lines:
            messagebox.showwarning("No corrected G-code", "Apply correction to G-code first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".gcode", filetypes=[("G-code","*.gcode"),("All files","*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding='utf-8') as f:
                for ln in self.corrected_gcode_lines:
                    f.write(ln.rstrip() + "\n")
            messagebox.showinfo("Saved", f"Corrected G-code saved to {path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))


    # ------------------------- Apply Height Map to G-code -------------------------
    def apply_height_map_to_gcode(self):
        if not self.gcode_lines:
            messagebox.showwarning("No G-code", "Load a G-code file first.")
            return
        with self._al_lock:
            if self.al_heights is None or len(self.al_xs) == 0 or len(self.al_ys) == 0:
                messagebox.showwarning("No map", "No probe map available. Run probing first.")
                return
            xs = self.al_xs[:]
            ys = self.al_ys[:]
            hs = self.al_heights.copy()
            ref = self.al_ref_height if self.al_ref_height is not None else 0.0

        corrected = []
        cur_x = None
        cur_y = None
        absolute_mode = True
        for line in self.gcode_lines:
            ln = line.strip()
            if re.search(r"\bG90\b", ln):
                absolute_mode = True
            if re.search(r"\bG91\b", ln):
                absolute_mode = False
            x_m = re.search(r"X(-?\d+\.?\d*)", ln, flags=re.IGNORECASE)
            y_m = re.search(r"Y(-?\d+\.?\d*)", ln, flags=re.IGNORECASE)
            z_m = re.search(r"Z(-?\d+\.?\d*)", ln, flags=re.IGNORECASE)
            if x_m:
                cur_x = float(x_m.group(1))
            if y_m:
                cur_y = float(y_m.group(1))
            if z_m:
                z_val = float(z_m.group(1))
                lookup_x = cur_x
                lookup_y = cur_y
                if lookup_x is None or lookup_y is None:
                    corrected.append(line)
                    continue
                surf_h = self._get_height_at(lookup_x, lookup_y, xs, ys, hs)
                if surf_h is None or np.isnan(surf_h):
                    corrected.append(line)
                    continue
                delta = surf_h - ref
                new_z = z_val + delta
                new_line = re.sub(r"(Z)-?\d+\.?\d*", lambda m: f"Z{new_z:.6f}", line, flags=re.IGNORECASE)
                corrected.append(new_line)
            else:
                corrected.append(line)
        self.corrected_gcode_lines = corrected
        messagebox.showinfo("Applied", "Height map corrections applied to loaded G-code (in-memory). Use 'Save Corrected G-code' to write to file.")
        self._log("Auto-level: corrections applied (in-memory)")

    def save_corrected_gcode(self):
        if not self.corrected_gcode_lines:
            messagebox.showwarning("No corrected G-code", "Apply correction to G-code first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".gcode", filetypes=[("G-code","*.gcode"),("All files","*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding='utf-8') as f:
                for ln in self.corrected_gcode_lines:
                    f.write(ln.rstrip() + "\n")
            messagebox.showinfo("Saved", f"Corrected G-code saved to {path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))

    def _get_height_at(self, x, y, xs, ys, hs):
        if len(xs) < 2 or len(ys) < 2:
            return float('nan')
        nx = len(xs)
        ny = len(ys)
        if x <= xs[0]:
            ix = 0
            fx = 0.0
        elif x >= xs[-1]:
            ix = nx - 2
            fx = 1.0
        else:
            ix = max(0, min(nx - 2, int(floor((x - xs[0]) / (xs[-1] - xs[0] + 1e-12) * (nx - 1)))))
            while ix + 1 < nx and xs[ix+1] < x:
                ix += 1
            while ix > 0 and xs[ix] > x:
                ix -= 1
            denom = xs[ix+1] - xs[ix] if xs[ix+1] != xs[ix] else 1e-12
            fx = (x - xs[ix]) / denom
        if y <= ys[0]:
            iy = 0
            fy = 0.0
        elif y >= ys[-1]:
            iy = ny - 2
            fy = 1.0
        else:
            iy = max(0, min(ny - 2, int(floor((y - ys[0]) / (ys[-1] - ys[0] + 1e-12) * (ny - 1)))))
            while iy + 1 < ny and ys[iy+1] < y:
                iy += 1
            while iy > 0 and ys[iy] > y:
                iy -= 1
            denomy = ys[iy+1] - ys[iy] if ys[iy+1] != ys[iy] else 1e-12
            fy = (y - ys[iy]) / denomy
        try:
            z00 = hs[iy, ix]
            z10 = hs[iy, ix+1] if ix+1 < hs.shape[1] else z00
            z01 = hs[iy+1, ix] if iy+1 < hs.shape[0] else z00
            z11 = hs[iy+1, ix+1] if (iy+1 < hs.shape[0] and ix+1 < hs.shape[1]) else z00
            corner_vals = [z00, z10, z01, z11]
            if any(np.isnan(corner_vals)):
                nonnan = [v for v in corner_vals if not np.isnan(v)]
                if nonnan:
                    return float(np.mean(nonnan))
                else:
                    return float('nan')
            z0 = z00 * (1 - fx) + z10 * fx
            z1 = z01 * (1 - fx) + z11 * fx
            z = z0 * (1 - fy) + z1 * fy
            return float(z)
        except Exception:
            return float('nan')
            
            




# ------------------------- Run the App -------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = CNCSenderApp(root)
    root.mainloop()
