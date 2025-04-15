import os
import tkinter as tk
from tkinter import filedialog, messagebox
from typing import Any, Dict, List, Optional
import plotly.subplots as sp

import customtkinter as ctk
import log_parser as lp
import plotly.graph_objs as go

# sets the defaults for the window
ctk.set_appearance_mode("Dark")  # Options: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme("green")  # Options: "blue" (standard), "green", "dark-blue"

PAGE_HEIGHT = 900  # Total default page height in pixels
MIN_SUBPLOT_HEIGHT = PAGE_HEIGHT // 3

# flag for animated marker
ENABLE_ANIMATION = False

def plot_all_selected_fields(
    file_path: str,
    schema: Dict[str, Any],
    selected_fields_by_message: Dict[str, List[str]],
    log_start_time: Optional[str] = None,
    plot_mode: str = "auto",
) -> None:
    n_subplots = len(selected_fields_by_message)
    if n_subplots == 0:
        print("[!] No fields selected to plot.")
        return

    # we're going to set all fields from a unique message on their own plot
    fig = sp.make_subplots(
        rows=n_subplots,
        cols=1,
        shared_xaxes=False,
        vertical_spacing=0.08,
        subplot_titles=[f"Message {msg_id}" for msg_id in selected_fields_by_message]
    )

    # get the actual log data
    try:
        msg_ids_int = [int(mid, 16) for mid in selected_fields_by_message.keys()]
        frames, log_start_time = lp.parse_log(file_path, msg_ids_int)
    except Exception as e:
        print(f"[!] Failed to parse messages: {e}")
        return

    for i, (msg_id_str, fields) in enumerate(selected_fields_by_message.items()):
        row = i + 1
        msg_id = int(msg_id_str, 16)

        if msg_id not in frames:
            print(f"[i] No data found for {msg_id_str}")
            continue

        df = frames[msg_id]
        if df.empty:
            print(f"[i] No data found for {msg_id_str}")
            continue

        is_trajectory_2d = set(["pos_x", "pos_y"]).issubset(fields)
        is_trajectory_3d = set(["pos_x", "pos_y", "pos_z"]).issubset(fields)

        if plot_mode in ("trajectory", "auto") and is_trajectory_3d:
            static_trace = go.Scatter3d(
                x=df["pos_x"], y=df["pos_y"], z=df["pos_z"],
                mode="lines", name=f"{msg_id_str} Path"
            )
            frames_list = [
                go.Frame(data=[
                    static_trace,
                    go.Scatter3d(
                        
                        x=[df["pos_x"].iloc[i]],
                        y=[df["pos_y"].iloc[i]],
                        z=[df["pos_z"].iloc[i]],
                        mode="markers",
                        marker=dict(size=5, color="red"),
                        name="Moving Marker",
                        uid="moving-marker"
                    )
                ]) for i in range(len(df))
            ]

            sliders = [{
                "steps": [{
                    "method": "animate",
                    "args": [[f.name], {"mode": "immediate", "frame": {"duration": 50}, "transition": {"duration": 0}}],
                    "label": str(i)
                } for i, f in enumerate(frames_list)],
                "active": 0
            }]

            fig3d = go.Figure(
                data=[static_trace, frames_list[0].data[0]],
                frames=frames_list
            )
            fig3d.update_layout(
                title=f"3D Trajectory — {msg_id_str}",
                scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z"),
                updatemenus=[{
                    "type": "buttons",
                    "buttons": [{
                        "label": "Play",
                        "method": "animate",
                        "args": [None, {"frame": {"duration": 50, "redraw": True}, "fromcurrent": True}]
                    }]
                }],
                sliders=sliders
            )
            fig3d.write_html(f"animated_trajectory_3d_{msg_id_str}.html")
            fig3d.show()
            continue
            continue

        elif plot_mode in ("trajectory", "auto") and is_trajectory_2d:
            if ENABLE_ANIMATION:
                static_trace = go.Scatter(
                    x=df["pos_x"], y=df["pos_y"],
                    mode="lines", name=f"{msg_id_str} Path"
                )
                frames_list = [
                    go.Frame(data=[
                        static_trace,
                        go.Scatter(
                            
                            x=[df["pos_x"].iloc[i]],
                            y=[df["pos_y"].iloc[i]],
                            mode="markers",
                            marker=dict(size=7, color="red"),
                            name="Moving Marker",
                            uid="moving-marker"
                        )
                    ]) for i in range(len(df))
                ]

                fig2d = go.Figure(
                    data=[static_trace, frames_list[0].data[0]],
                    frames=frames_list
                )
                fig2d.update_layout(
                    title=f"2D Trajectory — {msg_id_str}",
                    xaxis_title="pos_x", yaxis_title="pos_y",
                    updatemenus=[{
                        "type": "buttons",
                        "buttons": [{
                            "label": "Play",
                            "method": "animate",
                            "args": [None, {"frame": {"duration": 50, "redraw": True}, "fromcurrent": True}]
                        }]
                    }],
                    sliders=[{
                        "steps": [{
                            "method": "animate",
                            "args": [[f.name], {"mode": "immediate", "frame": {"duration": 50}, "transition": {"duration": 0}}],
                            "label": str(i)
                        } for i, f in enumerate(frames_list)],
                        "active": 0
                    }]
                )
                fig2d.write_html(f"animated_trajectory_2d_{msg_id_str}.html")
                fig2d.show()
                continue
            else:
                fig.add_trace(
                    go.Scatter(
                        x=df["pos_x"], y=df["pos_y"],
                        mode="lines+markers", name=f"{msg_id_str} XY Trajectory"
                    ), row=row, col=1
                )
        else:
            for field in fields:
                if field not in df.columns:
                    print(f"[!] Field {field} missing in {msg_id_str}")
                    continue
                fig.add_trace(
                    go.Scatter(
                        x=df.index, y=df[field],
                        mode="lines+markers", name=f"{msg_id_str}: {field}"
                    ), row=row, col=1
                )

    total_height = max(PAGE_HEIGHT, n_subplots * MIN_SUBPLOT_HEIGHT)

    fig.update_layout(
        height=total_height,
        title_text=f"Diagnostic Log Visualization{f' — {log_start_time}' if log_start_time else ''}",
        showlegend=True
    )
    fig.show()

class LogPlotterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Log Visualizer")
        self.root.geometry("800x600")
        
        # Set minimum size
        self.root.minsize(650, 500)
        
        self.schema = None
        self.frames = {}
        self.selected_message = tk.StringVar()
        self.selected_fields = []
        self.current_file = tk.StringVar(value="No file selected")
        self.selected_fields_by_message = {}

        self.target_log: str
        
        # Build the modern UI
        self.build_ui()
        
    def build_ui(self):
        # Create main container with padding
        self.main_frame = ctk.CTkFrame(self.root)
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        # Top section - Header and Load button in a frame
        self.header_frame = ctk.CTkFrame(self.main_frame)
        self.header_frame.pack(fill="x", padx=10, pady=(0, 20))
        
        # App title
        self.title_label = ctk.CTkLabel(
            self.header_frame, 
            text="Log Data Visualizer", 
            font=ctk.CTkFont(size=24, weight="bold")
        )
        self.title_label.pack(side="left", padx=10, pady=10)
        
        # File selection section
        self.file_frame = ctk.CTkFrame(self.main_frame)
        self.file_frame.pack(fill="x", padx=10, pady=(0, 20))
        
        self.load_button = ctk.CTkButton(
            self.file_frame,
            text="Load Log File",
            font=ctk.CTkFont(size=13),
            height=38,
            command=self.load_log
        )
        self.load_button.pack(side="left", padx=10, pady=10)
        
        self.file_label = ctk.CTkLabel(
            self.file_frame,
            textvariable=self.current_file,
            font=ctk.CTkFont(size=13)
        )
        self.file_label.pack(side="left", padx=10, pady=10, fill="x", expand=True)
        
        # Create two-column layout
        self.content_frame = ctk.CTkFrame(self.main_frame)
        self.content_frame.pack(fill="both", expand=True, padx=10, pady=(0, 20))
        self.content_frame.grid_columnconfigure(0, weight=1)
        self.content_frame.grid_columnconfigure(1, weight=3)
        
        # Left panel - Message selection
        self.left_panel = ctk.CTkFrame(self.content_frame)
        self.left_panel.grid(row=0, column=0, padx=(0, 10), pady=0, sticky="nsew")
        
        self.message_label = ctk.CTkLabel(
            self.left_panel,
            text="Message ID",
            font=ctk.CTkFont(size=16, weight="bold")
        )
        self.message_label.pack(anchor="w", padx=15, pady=(15, 5))
        
        self.message_dropdown = ctk.CTkComboBox(
            self.left_panel,
            variable=self.selected_message,
            state="readonly",
            values=["No messages loaded"],
            height=32,
            command=self.update_fields
        )
        self.message_dropdown.pack(fill="x", padx=15, pady=(5, 15))
        
        # Placeholder for message stats
        self.stats_label = ctk.CTkLabel(
            self.left_panel,
            text="",
            font=ctk.CTkFont(size=13)
        )
        self.stats_label.pack(anchor="w", padx=15, pady=(0, 15))
        
        # Right panel - Fields selection
        self.right_panel = ctk.CTkFrame(self.content_frame)
        self.right_panel.grid(row=0, column=1, padx=0, pady=0, sticky="nsew")
        
        self.fields_label = ctk.CTkLabel(
            self.right_panel,
            text="Available Fields",
            font=ctk.CTkFont(size=16, weight="bold")
        )
        self.fields_label.pack(anchor="w", padx=15, pady=(15, 5))
        
        # Helper text
        self.helper_label = ctk.CTkLabel(
            self.right_panel,
            text="Hold Ctrl to select multiple fields",
            font=ctk.CTkFont(size=12),
            text_color=("gray50", "gray70")  # (light mode, dark mode)
        )
        self.helper_label.pack(anchor="w", padx=15, pady=(0, 5))
        
        # Create a frame for the listbox and scrollbar
        self.fields_container = ctk.CTkFrame(self.right_panel)
        self.fields_container.pack(fill="both", expand=True, padx=15, pady=(0, 15))
        
        # We need to use a traditional Listbox because CTk doesn't have a multi-select listbox
        # But we style it to match our theme
        self.fields_listbox = tk.Listbox(
            self.fields_container,
            selectmode="multiple",
            exportselection=False,
            bg=self.calculate_listbox_bg_color(),
            fg=self.calculate_listbox_fg_color(),
            font=("Helvetica", 12),
            relief="flat",
            borderwidth=0,
            highlightthickness=0,
            selectbackground="#1f6aa5"
        )
        self.fields_listbox.pack(side="left", fill="both", expand=True)
        
        # Add a scrollbar that matches the theme
        self.scrollbar = ctk.CTkScrollbar(
            self.fields_container,
            command=self.fields_listbox.yview
        )
        self.scrollbar.pack(side="right", fill="y")
        self.fields_listbox.config(yscrollcommand=self.scrollbar.set)
        
        # Bottom section - Plot button
        self.plot_button = ctk.CTkButton(
            self.main_frame,
            text="Plot Selected Fields",
            font=ctk.CTkFont(size=15, weight="bold"),
            height=40,
            command=self.plot_fields
        )
        self.plot_button.pack(pady=(0, 10))
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ctk.CTkLabel(
            self.root,
            textvariable=self.status_var,
            font=ctk.CTkFont(size=12),
            anchor="w",
            height=25
        )
        self.status_bar.pack(fill="x", padx=2, pady=2)
        
        # Register theme change detection to update listbox colors
        # self.root.bind("<Configure>", self.check_appearance_mode_change)
        # self.current_appearance = ctk.get_appearance_mode()
    
    def calculate_listbox_bg_color(self):
        """Determine appropriate listbox background color based on theme"""
        appearance = ctk.get_appearance_mode()
        if appearance == "Dark":
            return "#2b2b2b"  # Dark background for dark mode
        else:
            return "#f9f9f9"  # Light background for light mode
    
    def calculate_listbox_fg_color(self):
        """Determine appropriate listbox text color based on theme"""
        appearance = ctk.get_appearance_mode()
        if appearance == "Dark":
            return "#dcddde"  # Light text for dark mode
        else:
            return "#1a1a1a"  # Dark text for light mode
    
    # def check_appearance_mode_change(self, event=None):
    #     """Check if appearance mode has changed and update listbox colors"""
    #     current_appearance = ctk.get_appearance_mode()
    #     if current_appearance != self.current_appearance:
    #         self.current_appearance = current_appearance
    #         self.fields_listbox.config(
    #             bg=self.calculate_listbox_bg_color(),
    #             fg=self.calculate_listbox_fg_color()
    #         )
    
    # def change_appearance_mode(self, new_appearance_mode):
    #     """Change the app's appearance mode"""
    #     ctk.set_appearance_mode(new_appearance_mode)
    #     # Update listbox colors
    #     self.fields_listbox.config(
    #         bg=self.calculate_listbox_bg_color(),
    #         fg=self.calculate_listbox_fg_color()
    #     )

    def load_log(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("Binary Log", "*.bin"), ("All Files", "*.*")]
        )
        if not file_path:
            return
            
        self.status_var.set("Loading log file...")
        self.root.update()
        
        try:
            self.schema = lp.load_schema()
            message_ids = sorted(self.schema.keys())  # e.g., ["0x01", "0x02", ...]
            message_names = [msg["name"] for msg in self.schema.values()]
            self.message_dropdown.configure(values=message_names)
            
            # Update current file display
            file_name = os.path.basename(file_path)
            self.current_file.set(file_name)

            # hold file name for log parsing target
            self.current_log = file_path
            
            if message_ids:
                self.selected_message.set(self.schema[message_ids[0]]["name"])
                self.update_fields(self.schema[message_ids[0]]["name"])
                
            self.status_var.set(f"Loaded {len(self.frames)} message types from {file_name}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load log file: {str(e)}")
            self.status_var.set("Error loading file")

    def update_fields(self, msg_name=None):
        if msg_name is None:
            msg_name = self.selected_message.get()

        msg_id = next((key for key, value in self.schema.items() if value["name"] == msg_name), None)
        self.selected_message.set(self.schema[msg_id]["name"])

        if not msg_id:
            self.status_var.set(f"Could not find message ID for {msg_name}")
            return

        self.fields_listbox.delete(0, tk.END)

        if msg_id and msg_id in self.schema:
            self.status_var.set("Loading fields...")
            for field in self.schema[msg_id]["fields"]:
                self.fields_listbox.insert(tk.END, field)

            # Restore previous selections if any
            if msg_id in self.selected_fields_by_message:
                saved_fields = self.selected_fields_by_message[msg_id]
                for i, field in enumerate(self.schema[msg_id]["fields"]):
                    if field in saved_fields:
                        self.fields_listbox.selection_set(i)
            
    def plot_fields(self):
        message_name = self.selected_message.get()
        msg_id = next((id for id, value in self.schema.items() if value["name"] == message_name), None)
        if not msg_id:
            messagebox.showerror("Error", "Message ID not found.")
            return

        selected_indices = self.fields_listbox.curselection()
        if not selected_indices:
            messagebox.showinfo("No Selection", "Please select at least one field to plot.")
            return

        selected_columns = [self.fields_listbox.get(i) for i in selected_indices]
        self.selected_fields_by_message[msg_id] = selected_columns

        self.status_var.set("Plotting all selected fields...")
        self.root.update()

        try:
            plot_all_selected_fields(
                file_path=self.current_log,
                schema=self.schema,
                selected_fields_by_message=self.selected_fields_by_message
            )
            self.status_var.set("Plotting complete.")
        except Exception as e:
            messagebox.showerror("Plot Error", f"Failed to create plot: {str(e)}")
            self.status_var.set("Error creating plot")


# Run GUI
if __name__ == "__main__":
    root = ctk.CTk()
    app = LogPlotterGUI(root)
    root.mainloop()