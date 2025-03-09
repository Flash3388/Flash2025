import tkinter as tk
from tkinter import font
from PIL import Image, ImageTk
import pyvjoy
import os
import sys
import time
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class CodeChangeHandler(FileSystemEventHandler):
    """Watches for changes in the script file and triggers reload"""
    def __init__(self, script_path, callback):
        self.script_path = os.path.abspath(script_path)
        self.callback = callback
        self.last_modified = os.path.getmtime(self.script_path)

    def on_modified(self, event):
        if not event.is_directory and event.src_path == self.script_path:
            current_modified = os.path.getmtime(self.script_path)
            if current_modified > self.last_modified:
                self.last_modified = current_modified
                print(f"Code change detected in {os.path.basename(self.script_path)}. Reloading...")
                self.callback()

class RobotControlUI:
    def __init__(self, root, auto_reload=True):
        self.root = root
        self.root.title("Flash 3388 - Robot Control Interface (FRC 2025)")
        self.root.geometry("1280x800")  # HD resolution with extra height for the image
        self.root.configure(bg="#1e272e")  # Dark background
        
        # Track fullscreen state
        self.is_fullscreen = False
        
        # Setup auto-reload if enabled
        self.auto_reload = auto_reload
        if self.auto_reload:
            self.setup_auto_reload()
        
        # Initialize two vJoy devices (joysticks)
        self.joystick1 = None  # Will control field buttons and team selection
        self.joystick2 = None  # Will control elevator, floppy and gripper
        
        try:
            self.joystick1 = pyvjoy.VJoyDevice(1)  # Device number 1
            self.vjoy1_connected = True
            print("vJoy Device 1 Connected!!")
        except Exception as e:
            self.vjoy1_connected = False
            print(f"Error connecting to vJoy Device 1: {e}")
            
        try:
            self.joystick2 = pyvjoy.VJoyDevice(2)  # Device number 2
            self.vjoy2_connected = True
            print("vJoy Device 2 Connected!!")
        except Exception as e:
            self.vjoy2_connected = False
            print(f"Error connecting to vJoy Device 2: {e}")
        
        # Define Unicode icons for buttons
        self.icons = {
            "up": "↑",           # Up arrow
            "down": "↓",         # Down arrow
            "up_double": "⇑",    # Double up arrow
            "down_double": "⇓",  # Double down arrow
            "left": "←",         # Left arrow
            "right": "→",        # Right arrow
            "in": "⟳",          # Algee in (for motors)
            "out": "⟳",         # Algee out (for motors)
            "gripper_open": "⊃", # Open gripper
            "gripper_close": "⊂" # Close gripper
        }
        
        # Font settings
        self.title_font = font.Font(family="Arial", size=22, weight="bold")
        self.button_font = font.Font(family="Arial", size=16, weight="bold")
        self.status_font = font.Font(family="Arial", size=14)
        self.field_button_font = font.Font(family="Arial", size=10, weight="bold")
        self.icon_font = font.Font(family="Arial", size=24, weight="bold")  # Larger font for icons
        # New font for selector buttons
        self.selector_font = font.Font(family="Arial", size=12, weight="bold")
        
        # ===== Header section with fullscreen toggle =====
        self.header_frame = tk.Frame(root, bg="#1e272e")
        self.header_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Create main header
        self.header = tk.Label(
            self.header_frame,
            text="Flash 3388 - Robot Control System",
            font=("Arial", 26, "bold"),
            bg="#1e272e",
            fg="white"
        )
        self.header.pack(side=tk.LEFT, pady=5)
        
        # Fullscreen toggle button
        self.fullscreen_btn = tk.Button(
            self.header_frame,
            text="מסך מלא " + self.icons["up_double"],
            font=("Arial", 12),
            bg="#f39c12",
            fg="white",
            padx=10,
            command=self.toggle_fullscreen
        )
        self.fullscreen_btn.pack(side=tk.RIGHT, pady=5, padx=10)
        
        # ===== Main content container - split into two halves =====
        self.content_frame = tk.Frame(root, bg="#1e272e")
        self.content_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Configure the grid to have two columns with custom width ratio
        self.content_frame.columnconfigure(0, weight=1)  # Left half - Field
        self.content_frame.columnconfigure(1, weight=4)  # Right half - Controls
        
        # ===== Left half - Field Image Area =====
        self.field_frame = tk.Frame(self.content_frame, bg="#1e272e", padx=5, pady=5)
        self.field_frame.grid(row=0, column=0, sticky="nsew")
        
        # Create Canvas for field image and buttons
        self.canvas = tk.Canvas(self.field_frame, bg="#1e272e", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Add mouse movement tracking
        self.canvas.bind("<Motion>", self.track_mouse)
        self.canvas.bind("<Button-1>", self.log_click_position)
        
        # ===== Right half - Control Buttons and Selectors =====
        self.right_frame = tk.Frame(self.content_frame, bg="#1e272e", padx=5, pady=5)
        self.right_frame.grid(row=0, column=1, sticky="nsew")
        
        # Configure the right frame to have two columns
        self.right_frame.columnconfigure(0, weight=1)  # Selector column
        self.right_frame.columnconfigure(1, weight=4)  # Controls column
       
        # ===== Selector Area (New) =====
        self.selector_frame = tk.Frame(self.right_frame, bg="#1e272e", padx=5, pady=5, bd=2, relief=tk.GROOVE)
        self.selector_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Team color selector
        self.team_color_frame = tk.LabelFrame(
            self.selector_frame, 
            text="צבע קבוצה", 
            font=self.selector_font,
            bg="#1e272e",
            fg="white",
            padx=10, 
            pady=10,
            labelanchor="ne"
        )
        self.team_color_frame.pack(fill=tk.X, pady=10, padx=5)
        
        # Blue button
        self.blue_btn = tk.Button(
            self.team_color_frame,
            text="כחול",
            font=self.selector_font,
            bg="#0000ff",
            fg="white",
            width=8,
            height=2,
            command=lambda: self.select_team_color("blue")
        )
        self.blue_btn.pack(fill=tk.X, pady=5)
        
        # Red button
        self.red_btn = tk.Button(
            self.team_color_frame,
            text="אדום",
            font=self.selector_font,
            bg="#ff0000",
            fg="white",
            width=8,
            height=2,
            command=lambda: self.select_team_color("red")
        )
        self.red_btn.pack(fill=tk.X, pady=5)
        
        # Reef level selector
        self.reef_frame = tk.LabelFrame(
            self.selector_frame, 
            text="קומת ריף", 
            font=self.selector_font,
            bg="#1e272e",
            fg="white",
            padx=10, 
            pady=10,
            labelanchor="ne"
        )
        self.reef_frame.pack(fill=tk.X, pady=10, padx=5)
        
        # L1 button
        self.l1_btn = tk.Button(
            self.reef_frame,
            text="קומה 1 (מדף)",
            font=self.selector_font,
            bg="#00ff00",
            fg="black",
            width=8,
            height=2,
            command=lambda: self.select_reef_level("L1")
        )
        self.l1_btn.pack(fill=tk.X, pady=5)
        
        # L2 button
        self.l2_btn = tk.Button(
            self.reef_frame,
            text="קומה 2",
            font=self.selector_font,
            bg="#00ff00",
            fg="black",
            width=8,
            height=2,
            command=lambda: self.select_reef_level("L2")
        )
        self.l2_btn.pack(fill=tk.X, pady=5)
        
        # L3 button
        self.l3_btn = tk.Button(
            self.reef_frame,
            text="קומה 3",
            font=self.selector_font,
            bg="#00ff00",
            fg="black",
            width=8,
            height=2,
            command=lambda: self.select_reef_level("L3")
        )
        self.l3_btn.pack(fill=tk.X, pady=5)
        
        # L3A button
        self.l4_btn = tk.Button(
            self.reef_frame,
            text="אלגי + קומה 3",
            font=self.selector_font,
            bg="#00ff00",
            fg="black",
            width=8,
            height=2,
            command=lambda: self.select_reef_level("L3A")
        )
        self.l4_btn.pack(fill=tk.X, pady=5)


        self.automation_frame = tk.LabelFrame(
            self.selector_frame, 
            text="אוטומציה", 
            font=self.selector_font,
            bg="#1e272e",
            fg="white",
            padx=10, 
            pady=10,
            labelanchor="ne"
        )
        self.automation_frame.pack(fill=tk.X, pady=10, padx=5)

        # ===== Censel Outomation =====
        self.cancel_automation_btn = tk.Button(
            self.automation_frame,
            text="ביטול אוטומציה",
            font=self.selector_font,
            bg="#9b59b6",  # צבע סגול
            fg="white",
            width=8,
            height=2,
            command=lambda: self.press_button(11, "Cancel Automation", joystick_num=2)
        )
        self.cancel_automation_btn.pack(fill=tk.X, pady=5)

        
        # ===== Controls container =====
        self.controls_outer_frame = tk.Frame(self.right_frame, bg="#1e272e", padx=5, pady=5)
        self.controls_outer_frame.grid(row=0, column=1, sticky="nsew", padx=(350, 5))
        
        # Container for all control sections
        self.controls_container = tk.Frame(self.controls_outer_frame, bg="#1e272e")
        self.controls_container.pack(fill=tk.BOTH, expand=True)
        
        # Create three sections for the controls with equal size
        self.controls_container.rowconfigure(0, weight=1)  # Equal height for all rows
        self.controls_container.rowconfigure(1, weight=1)
        self.controls_container.rowconfigure(2, weight=1)
        self.controls_container.columnconfigure(0, weight=1)  # Full width
        
        # ===== 1. Elevator Control Section =====
        self.elevator_frame = tk.Frame(self.controls_container, bg="#1e272e", padx=20, pady=10, bd=2, relief=tk.GROOVE)
        self.elevator_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Elevator title
        self.elevator_title = tk.Label(
            self.elevator_frame,
            text=f"בקרת מעלית {self.icons['up']} {self.icons['down']}",
            font=self.title_font,
            bg="#1e272e",
            fg="#3498db"
        )
        self.elevator_title.pack(pady=(5, 15))
        
        # Button container for elevator buttons (horizontal layout)
        self.elevator_buttons_frame = tk.Frame(self.elevator_frame, bg="#1e272e")
        self.elevator_buttons_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.elevator_buttons_frame.columnconfigure(0, weight=1)
        self.elevator_buttons_frame.columnconfigure(1, weight=1)
        
        # Elevator buttons - starting from 1 on joystick 2
        self.elevator_up_btn = self.create_square_button(
            self.elevator_buttons_frame, 
            f"{self.icons['up_double']}\nהרם\nמעלית", 
            "#2980b9", 
            lambda: self.press_button(1, "Raise Elevator", joystick_num=2),
            row=0, column=0
        )
        
        self.elevator_down_btn = self.create_square_button(
            self.elevator_buttons_frame, 
            f"{self.icons['down_double']}\nהורד\nמעלית", 
            "#3498db", 
            lambda: self.press_button(2, "Lower Elevator", joystick_num=2),
            row=0, column=1
        )
        
        # ===== 2. Floppy Control Section =====
        self.floppy_frame = tk.Frame(self.controls_container, bg="#1e272e", padx=20, pady=10, bd=2, relief=tk.GROOVE)
        self.floppy_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        
        # Floppy header
        self.floppy_title = tk.Label(
            self.floppy_frame,
            text=f"בקרת פלופי {self.icons['in']}",
            font=self.title_font,
            bg="#1e272e",
            fg="#e74c3c"
        )
        self.floppy_title.pack(pady=(5, 15))
        
        # Button container for floppy buttons (2x2 grid)
        self.floppy_buttons_frame = tk.Frame(self.floppy_frame, bg="#1e272e")
        self.floppy_buttons_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Configure the grid for 2x2 layout
        for i in range(2):
            self.floppy_buttons_frame.rowconfigure(i, weight=1)
            self.floppy_buttons_frame.columnconfigure(i, weight=1)
        
        # Floppy buttons - continue numbers from 3 on joystick 2
        self.floppy_up_btn = self.create_square_button(
            self.floppy_buttons_frame, 
            f"{self.icons['up']}\nהרם\nפלופי", 
            "#c0392b", 
            lambda: self.press_button(3, "Raise Floppy", joystick_num=2),
            row=0, column=0
        )
        
        self.floppy_down_btn = self.create_square_button(
            self.floppy_buttons_frame, 
            f"{self.icons['down']}\nהורד\nפלופי", 
            "#e74c3c", 
            lambda: self.press_button(4, "Lower Floppy", joystick_num=2),
            row=0, column=1
        )
        
        self.floppy_start_btn = self.create_square_button(
            self.floppy_buttons_frame, 
            f"{self.icons['in']}\nהכנס\nכדור", 
            "#d35400", 
            lambda: self.press_button(5, "Start Floppy Motors", joystick_num=2),
            row=1, column=0
        )
        
        self.floppy_stop_btn = self.create_square_button(
            self.floppy_buttons_frame, 
            f"{self.icons['out']}\nהוצא\nכדור", 
            "#e67e22", 
            lambda: self.press_button(6, "Stop Floppy Motors", joystick_num=2),
            row=1, column=1
        )
        
        # ===== 3. Gripper Control Section =====
        self.gripper_frame = tk.Frame(self.controls_container, bg="#1e272e", padx=20, pady=10, bd=2, relief=tk.GROOVE)
        self.gripper_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        
        # Gripper header
        self.gripper_title = tk.Label(
            self.gripper_frame,
            text=f"בקרת גריפר {self.icons['gripper_open']} {self.icons['gripper_close']}",
            font=self.title_font,
            bg="#1e272e",
            fg="#27ae60"
        )
        self.gripper_title.pack(pady=(5, 15))
        
        # Button container for gripper buttons (2x2 grid)
        self.gripper_buttons_frame = tk.Frame(self.gripper_frame, bg="#1e272e")
        self.gripper_buttons_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Configure the grid for 2x2 layout
        for i in range(2):
            self.gripper_buttons_frame.rowconfigure(i, weight=1)
            self.gripper_buttons_frame.columnconfigure(i, weight=1)
        
        # Gripper buttons - continue numbers from 7 on joystick 2
       
        self.coral_out_btn = self.create_square_button(
            self.gripper_buttons_frame, 
            f"{self.icons['gripper_open']}\nהוצא\nקורל", 
            "#16a085", 
            lambda: self.press_button(8, "Pull Out Coral", joystick_num=2),
            row=1, column=0
        )
        
        self.coral_in_btn = self.create_square_button(
            self.gripper_buttons_frame, 
            f"{self.icons['gripper_close']}\nקבל\nקורל", 
            "#1abc9c", 
            lambda: self.press_button(7, "Pull in Coral", joystick_num=2),
            row=1, column=1
        )

        self.gripper_forward_btn = self.create_square_button(
            self.gripper_buttons_frame, 
            f"{self.icons['left']}\nגריפר\nקדימה (לריף)", 
            "#27ae60", 
            lambda: self.press_button(9, "Gripper Forward", joystick_num=2),
            row=0, column=0
        )
        
        self.gripper_back_btn = self.create_square_button(
            self.gripper_buttons_frame, 
            f"{self.icons['right']}\nגריפר\nאחורה (לפידר)", 
            "#2ecc71", 
            lambda: self.press_button(10, "Gripper Backward", joystick_num=2),
            row=0, column=1
        )        
        
        # ===== Status bar at the bottom =====
        self.status_frame = tk.Frame(root, bg="#1e272e", pady=5, bd=1, relief=tk.GROOVE)
        self.status_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=5)
        
        # Status indicator for joystick 1
        self.status_label1 = tk.Label(
            self.status_frame,
            text="Joystick 1: " + ("Connected" if self.vjoy1_connected else "Not Connected"),
            font=self.status_font,
            bg="#1e272e",
            fg="#3498db" if self.vjoy1_connected else "#e74c3c",
            padx=10
        )
        self.status_label1.pack(side=tk.LEFT, padx=10)
        
        # Status indicator for joystick 2
        self.status_label2 = tk.Label(
            self.status_frame,
            text="Joystick 2: " + ("Connected" if self.vjoy2_connected else "Not Connected"),
            font=self.status_font,
            bg="#1e272e",
            fg="#3498db" if self.vjoy2_connected else "#e74c3c",
            padx=10
        )
        self.status_label2.pack(side=tk.LEFT, padx=10)
        
        # Status label for commands
        self.command_label = tk.Label(
            self.status_frame,
            text="מערכת מחוברת ומוכנה",
            font=self.status_font,
            bg="#1e272e",
            fg="#3498db",
            padx=10
        )
        self.command_label.pack(side=tk.LEFT, padx=10)
        
        # Auto-reload status indicator
        if self.auto_reload:
            self.reload_label = tk.Label(
                self.status_frame,
                text="טעינת קוד דינמית",
                font=self.status_font,
                bg="#1e272e",
                fg="#f39c12",
                padx=10
            )
            self.reload_label.pack(side=tk.LEFT, padx=10)
        
        # Coordinates label
        self.coords_label = tk.Label(
            self.status_frame,
            text="מיקום: X: 0, Y: 0",
            font=self.status_font,
            bg="#1e272e",
            fg="#f39c12",
            padx=10
        )
        self.coords_label.pack(side=tk.RIGHT, padx=10)
        
        # Current coordinates information
        self.current_x = 0
        self.current_y = 0
        
        # Also bind Escape key to exit fullscreen
        self.root.bind("<Escape>", self.exit_fullscreen)
        
        # Initialize selected team color and reef level
        self.current_team_color = "red"  # Default is red
        self.current_reef_level = "L3"   # Default is L3
        
        # Set initial button states
        self.red_btn.config(relief=tk.SUNKEN, bg="#cc0000")  # Start with red selected
        self.l3_btn.config(relief=tk.SUNKEN, bg="#00cc00")   # Start with L1 selected
        
        # Load field image
        self.load_field_image("red")  # Default to red team
    
    def select_team_color(self, color):
        """Select the team color (blue or red)"""
        # Only make changes if the color is actually changing
        if self.current_team_color == color:
            return
            
        self.current_team_color = color
        
        # Update buttons to show which is selected
        if color == "blue":
            self.blue_btn.config(relief=tk.SUNKEN, bg="#0000cc")
            self.red_btn.config(relief=tk.RAISED, bg="#ff0000")
            self.command_label.config(text=f"צבע קבוצה נבחר - כחול")
            
            # Send vJoy button press for joystick 1, button 1 (blue team)
            if self.vjoy1_connected:
                self.press_button(22, "Blue Team Selected", joystick_num=1)
                
            # Reload the field image with blue version
            self.load_field_image("blue")
            
        else:
            self.blue_btn.config(relief=tk.RAISED, bg="#0000ff")
            self.red_btn.config(relief=tk.SUNKEN, bg="#cc0000")
            self.command_label.config(text=f"צבע קבוצה נבחר - אדום")
            
            # Send vJoy button press for joystick 1, button 2 (red team)
            if self.vjoy1_connected:
                self.press_button(23, "Red Team Selected", joystick_num=1)
                
            # Reload the field image with red version
            self.load_field_image("red")
        
        print(f"Team color changed to {color}")
    
    def select_reef_level(self, level):
        """Select the reef level (L1, L2, L3 or L3A)"""
        # Only make changes if the level is actually changing
        if self.current_reef_level == level:
            return
            
        self.current_reef_level = level
        
        # Update buttons to show which is selected
        self.l1_btn.config(relief=tk.RAISED, bg="#00ff00")
        self.l2_btn.config(relief=tk.RAISED, bg="#00ff00")
        self.l3_btn.config(relief=tk.RAISED, bg="#00ff00")
        self.l4_btn.config(relief=tk.RAISED, bg="#00ff00")
        
        # Send vJoy button press based on level
        if level == "L1":
            self.l1_btn.config(relief=tk.SUNKEN, bg="#00cc00")
            if self.vjoy1_connected:
                self.press_button(24, "Level L1 Selected", joystick_num=1)
        elif level == "L2":
            self.l2_btn.config(relief=tk.SUNKEN, bg="#00cc00")
            if self.vjoy1_connected:
                self.press_button(25, "Level L2 Selected", joystick_num=1)
        elif level == "L3":
            self.l3_btn.config(relief=tk.SUNKEN, bg="#00cc00")
            if self.vjoy1_connected:
                self.press_button(26, "Level L3 Selected", joystick_num=1)
        else:  # L3A
            self.l4_btn.config(relief=tk.SUNKEN, bg="#00cc00")
            if self.vjoy1_connected:
                self.press_button(27, "Level L3A Selected", joystick_num=1)
        
        self.command_label.config(text=f"Reef level set to {level}")
        print(f"Reef level changed to {level}")

    def load_field_image(self, team_color):
        """Load and display the field image with buttons based on team color"""
        try:
            # Debug print to verify function is being called
            print(f"Loading field image for team color: {team_color}")
            
            # Choose the correct image file based on team color
            field_image_path = f"field-{team_color}.png"
            
            # Debug print to verify file path
            print(f"Looking for image file: {field_image_path}")
            
            # Check if file exists
            if os.path.exists(field_image_path):
                print(f"Image file exists: {field_image_path}")
                
                # Load image 
                field_image = Image.open(field_image_path)
                
                # Calculate dimensions while maintaining aspect ratio
                # Keep the image size from the original code
                frame_width = 816  # Target width
                frame_height = 875  # Target height
                
                # Resize to fit in the frame while maintaining aspect ratio
                img_width, img_height = field_image.size
                aspect_ratio = img_width / img_height
                
                # Determine how to resize to fit in the frame
                if aspect_ratio > (frame_width / frame_height):
                    # Image is wider
                    new_width = frame_width
                    new_height = int(frame_width / aspect_ratio)
                else:
                    # Image is taller
                    new_height = frame_height
                    new_width = int(frame_height * aspect_ratio)
                
                # Resize the image
                field_image = field_image.resize((new_width, new_height), Image.LANCZOS)
                self.field_photo = ImageTk.PhotoImage(field_image)
                
                # Configure canvas size
                self.canvas.config(width=new_width, height=new_height)
                
                # Clear existing canvas items
                self.canvas.delete("all")
                
                # Calculate center position to center the image in the canvas
                x_pos = (new_width - self.field_photo.width()) // 2
                y_pos = (new_height - self.field_photo.height()) // 2
                
                # Display image in Canvas
                self.field_image_id = self.canvas.create_image(x_pos, y_pos, anchor="nw", image=self.field_photo)
                
                # Create buttons on the field based on team color
                self.create_field_buttons(team_color)
                
                print(f"Field image loaded successfully. Size: {new_width}x{new_height}")
            else:
                print(f"Error: Image file '{field_image_path}' not found.")
                # Display error message instead of image
                self.canvas.delete("all")
                self.canvas.config(width=600, height=600)
                self.canvas.create_text(
                    300, 300,
                    text=f"Cannot load field image.\nPlease save '{field_image_path}' in the same directory.",
                    font=self.status_font,
                    fill="#e74c3c"
                )
        except Exception as e:
            print(f"Error loading field image: {e}")
            # Display error message instead of image
            self.canvas.delete("all")
            self.canvas.config(width=600, height=300)
            self.canvas.create_text(
                300, 150,
                text=f"Error loading field image: {e}",
                font=self.status_font,
                fill="#e74c3c"
            )
    
    def toggle_fullscreen(self):
        """Toggle between fullscreen and normal window mode"""
        self.is_fullscreen = not self.is_fullscreen
        
        # Update the window state
        self.root.attributes('-fullscreen', self.is_fullscreen)
        
        # Update the button text
        if self.is_fullscreen:
            self.fullscreen_btn.config(text=f"מסך רגיל {self.icons['down_double']}")
        else:
            self.fullscreen_btn.config(text=f"מסך מלא {self.icons['up_double']}")
    
    def exit_fullscreen(self, event=None):
        """Exit fullscreen mode (can be triggered by Escape key)"""
        if self.is_fullscreen:
            self.is_fullscreen = False
            self.root.attributes('-fullscreen', False)
            self.fullscreen_btn.config(text=f"מסך מלא {self.icons['up_double']}")
    
    def setup_auto_reload(self):
        """Set up auto-reload functionality to monitor the script for changes"""
        try:
            # Get the current script path
            self.script_path = os.path.abspath(sys.argv[0])
            
            # Set up the file system event handler
            self.event_handler = CodeChangeHandler(self.script_path, self.reload_application)
            self.observer = Observer()
            self.observer.schedule(self.event_handler, os.path.dirname(self.script_path), recursive=False)
            self.observer.start()
            
            print(f"Auto-reload activated. Watching for changes to {os.path.basename(self.script_path)}")
        except Exception as e:
            print(f"Error setting up auto-reload: {e}")
    
    def reload_application(self):
        """Reload the application when code changes are detected"""
        try:
            print("Restarting application...")
            
            # Clean shutdown of the current application
            if hasattr(self, 'observer'):
                self.observer.stop()
            
            # Schedule restart after a short delay to allow for file operations to complete
            self.root.after(100, self._perform_restart)
        except Exception as e:
            print(f"Error during reload: {e}")
    
    def _perform_restart(self):
        """Actually perform the restart operation"""
        try:
            # Clean up resources
            self.root.destroy()
            
            # Start a new process with the same arguments
            python = sys.executable
            os.execl(python, python, *sys.argv)
        except Exception as e:
            print(f"Failed to restart application: {e}")
            # If restart fails, try to continue the current instance
            if not self.root.winfo_exists():
                print("Main window was destroyed. Cannot continue.")
                sys.exit(1)
    
    def track_mouse(self, event):
        """Track mouse position and display coordinates"""
        self.current_x = event.x
        self.current_y = event.y
        self.coords_label.config(text=f"Mouse position: X: {event.x}, Y: {event.y}")
    
    def log_click_position(self, event):
        """Record click position"""
        x, y = event.x, event.y
        print(f"Clicked at point: {x}, {y}")
    
    def create_field_buttons(self, team_color):
        """Create buttons on the field according to ID locations and team color"""
        # Debug print
        print(f"Creating field buttons for team color: {team_color}")
        
        # Dimensions and design options for field buttons
        button_width = 50   # Button width
        button_height = 30  # Button height
        
        # Button positions based on team color
        if team_color == "red":
            # Original red team button positions
            button_positions = [
                # Positions - adjust according to your specific image
                (78,  674,   '1R',  1, "#ff99cc"),   # ID 1R
                (113, 717,   '1C',  2, "#ff99cc"),   # ID 1C
                (142, 760,   '1L',  3, "#ff99cc"),   # ID 1L
                (639, 674,   '2L',  4, "#ff99cc"),   # ID 2L
                (610, 717,   '2C',  5, "#ff99cc"),   # ID 2C
                (573, 760,   '2R',  6, "#ff99cc"),   # ID 2R
                (653, 322,    '3',  7, "#ff99cc"),   # ID 3
                (526, 172,    '4',  8, "#ff99cc"),   # ID 4
                (191, 172,    '5',  9, "#ff99cc"),   # ID 5
                (267, 525,   '6R', 11, "#ff99cc"),   # ID 6R
                (240, 480,   '6L', 10, "#ff99cc"),   # ID 6L
                (391, 553,   '7R', 13, "#ff99cc"),   # ID 7R
                (327, 553,   '7L', 12, "#ff99cc"),   # ID 7L
                (479, 480,   '8R', 15, "#ff99cc"),   # ID 8R
                (452, 525,   '8L', 14, "#ff99cc"),   # ID 8L
                (452, 357,   '9R', 17, "#ff99cc"),   # ID 9R
                (479, 402,   '9L', 16, "#ff99cc"),   # ID 9L
                (391, 325,  '10L', 18, "#ff99cc"),   # ID 10R
                (327, 325,  '10R', 19, "#ff99cc"),   # ID 10L
                (267, 357,  '11L', 20, "#ff99cc"),   # ID 11R
                (240, 402,  '11R', 21, "#ff99cc"),   # ID 11L
            ]
        else:
            # Blue team button positions
            button_positions = [
                # Positions - using the provided list for blue team
                (78,  674,   '12R',  1, "#99ccff"),   # ID 12R
                (113, 717,   '12C',  2, "#99ccff"),   # ID 12C
                (142, 760,   '12L',  3, "#99ccff"),   # ID 12L
                (639, 674,   '13L',  4, "#99ccff"),   # ID 13L
                (610, 717,   '13C',  5, "#99ccff"),   # ID 13C
                (573, 760,   '13R',  6, "#99ccff"),   # ID 13R
                (653, 322,    '16',  7, "#99ccff"),   # ID 16
                (526, 172,    '14',  8, "#99ccff"),   # ID 14
                (191, 172,    '15',  9, "#99ccff"),   # ID 15
                (267, 525,   '17R', 10, "#99ccff"),   # ID 17R
                (240, 480,   '17L', 11, "#99ccff"),   # ID 17L
                (391, 553,   '18R', 12, "#99ccff"),   # ID 18R
                (327, 553,   '18L', 13, "#99ccff"),   # ID 18L
                (479, 480,   '19R', 14, "#99ccff"),   # ID 19R
                (452, 525,   '19L', 15, "#99ccff"),   # ID 19L
                (452, 357,   '20R', 16, "#99ccff"),   # ID 20R
                (479, 402,   '20L', 17, "#99ccff"),   # ID 20L
                (391, 325,   '21L', 18, "#99ccff"),   # ID 21R
                (327, 325,   '21R', 19, "#99ccff"),   # ID 21L
                (267, 357,   '22L', 20, "#99ccff"),   # ID 22R
                (240, 402,   '22R', 21, "#99ccff"),   # ID 22L
            ]
        
        print(f"Creating {len(button_positions)} buttons for {team_color} team")
        
        # Create buttons with transparent background
        for x, y, button_label, button_id, color in button_positions:
            # Calculate button corners
            x1 = x - button_width // 2
            y1 = y - button_height // 2
            x2 = x + button_width // 2
            y2 = y + button_height // 2
            
            # Create button background
            button_bg = self.canvas.create_rectangle(
                x1, y1, x2, y2,
                fill=color,
                outline="#ffffff",
                width=2
            )
            
            # Create button text
            button_text = self.canvas.create_text(
                x, y,
                text=f"ID {button_label}",
                font=self.field_button_font,
                fill="#000000"
            )
            
            # Add click events to background and text - all field buttons use joystick 1
            self.canvas.tag_bind(button_bg, "<Button-1>", 
                             lambda event, bid=button_id: self.press_button(bid, f"Field button {bid}", joystick_num=1))
            self.canvas.tag_bind(button_text, "<Button-1>", 
                             lambda event, bid=button_id: self.press_button(bid, f"Field button {bid}", joystick_num=1))
    
    def create_square_button(self, parent, text, color, command, row=0, column=0):
        """Create square-shaped styled button with proper grid placement"""
        btn = tk.Button(
            parent,
            text=text,
            font=self.button_font,
            bg=color,
            fg="white",
            activebackground=self.lighten_color(color),
            activeforeground="white",
            relief=tk.RAISED,
            bd=3,
            width=3,  # Width in text units
            height=3,  # Height in text units
            wraplength=90,  # Allow text wrapping
            command=command
        )
        
        # Place in the grid with padding to create space between buttons
        btn.grid(row=row, column=column, padx=10, pady=10, sticky="nsew")
        
        return btn
    
    def lighten_color(self, hex_color):
        """Lighten a hexadecimal color"""
        # Convert from hexadecimal to RGB
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        
        # Lighten the color
        factor = 1.2
        r = min(255, int(r * factor))
        g = min(255, int(g * factor))
        b = min(255, int(b * factor))
        
        # Return to hexadecimal
        return f'#{r:02x}{g:02x}{b:02x}'
    
    def press_button(self, button_num, action_name, joystick_num=1):
        """Press a button on specified joystick"""
        # Debug information
        print(f"Pressing button {button_num} on joystick {joystick_num}: {action_name}")
        
        # Choose the correct joystick based on joystick_num
        if joystick_num == 1:
            joystick = self.joystick1
            joystick_connected = self.vjoy1_connected
        else:  # joystick_num == 2
            joystick = self.joystick2
            joystick_connected = self.vjoy2_connected
        
        if not joystick_connected:
            self.command_label.config(text=f"Error: vJoy Device {joystick_num} not connected", fg="#e74c3c")
            print(f"Error: vJoy Device {joystick_num} not connected")
            return
            
        try:
            # Press
            joystick.set_button(button_num, 1)
            self.command_label.config(text=f"Command: {action_name} (Joy{joystick_num} Btn{button_num})", fg="#3498db")
            
            # Release after 100 milliseconds
            self.root.after(100, lambda: self.release_button(button_num, action_name, joystick_num))
        except Exception as e:
            self.command_label.config(text=f"Error: {e}", fg="#e74c3c")
            print(f"Error pressing button {button_num} on joystick {joystick_num}: {e}")
    
    def release_button(self, button_num, action_name, joystick_num=1):
        """Release button on specified joystick"""
        # Debug information
        print(f"Releasing button {button_num} on joystick {joystick_num}: {action_name}")
        
        # Choose the correct joystick based on joystick_num
        if joystick_num == 1:
            joystick = self.joystick1
            joystick_connected = self.vjoy1_connected
        else:  # joystick_num == 2
            joystick = self.joystick2
            joystick_connected = self.vjoy2_connected
        
        if not joystick_connected:
            return
            
        try:
            joystick.set_button(button_num, 0)
            self.command_label.config(text=f"Command executed: {action_name}", fg="#27ae60")
        except Exception as e:
            self.command_label.config(text=f"Error releasing button: {e}", fg="#e74c3c")
            print(f"Error releasing button {button_num} on joystick {joystick_num}: {e}")
    
    def on_closing(self):
        """Cleanup when closing the application"""
        if hasattr(self, 'observer'):
            self.observer.stop()
            self.observer.join()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlUI(root, auto_reload=True)  # Set to True to enable auto-reload
    
    # Set the closing protocol
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()