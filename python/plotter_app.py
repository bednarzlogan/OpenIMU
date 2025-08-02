
import tkinter as tk
import customtkinter as ctk
from tkinter import filedialog, messagebox
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import log_parser as lp
import sys
import io

ctk.set_default_color_theme("dark-blue")
ctk.set_appearance_mode("dark")


class TextRedirector(io.StringIO):
    def __init__(self, widget):
        super().__init__()
        self.widget = widget

    def write(self, s):
        self.widget.configure(state="normal")
        self.widget.insert("end", s)
        self.widget.see("end")
        self.widget.configure(state="disabled")

    def flush(self):
        pass


class SignalPlotterApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Binary Log Signal Viewer")
        self.geometry("1200x800")

        self.schema = {}
        self.frames = {}
        self.current_log = None

        self.top_frame = ctk.CTkFrame(self)
        self.top_frame.pack(side="top", fill="both", expand=True)

        self.bottom_frame = ctk.CTkFrame(self)
        self.bottom_frame.pack(side="bottom", fill="x")

        self.sidebar_frame = ctk.CTkFrame(self.top_frame, width=300, corner_radius=0)
        self.sidebar_frame.pack(side="left", fill="y")

        self.plot_frame = ctk.CTkFrame(self.top_frame)
        self.plot_frame.pack(side="right", expand=True, fill="both")

        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.status_var = tk.StringVar()
        self.status_label = ctk.CTkLabel(self.bottom_frame, textvariable=self.status_var)
        self.status_label.pack(side="bottom", fill="x")

        self.log_text = tk.Text(self.bottom_frame, height=8, wrap="word", bg="#1e1e1e", fg="#ffffff", font=("Courier", 10))
        self.log_text.pack(padx=10, pady=(0, 10), fill="x")
        self.log_text.configure(state="disabled")

        sys.stdout = TextRedirector(self.log_text)
        sys.stderr = TextRedirector(self.log_text)

        self.current_file = tk.StringVar()
        self.selected_message = tk.StringVar()

        self.init_sidebar_controls()

    def init_sidebar_controls(self):
        label = ctk.CTkLabel(self.sidebar_frame, text="Session Controls", font=("Arial", 16))
        label.pack(pady=(20, 10))

        self.load_btn = ctk.CTkButton(self.sidebar_frame, text="Load Log File", command=self.load_log_file)
        self.load_btn.pack(pady=5, padx=10, fill="x")

        self.message_dropdown = ctk.CTkOptionMenu(self.sidebar_frame, variable=self.selected_message, values=[], command=self.update_fields)
        self.message_dropdown.pack(pady=10, padx=10, fill="x")

        self.x_field_label = ctk.CTkLabel(self.sidebar_frame, text="X Field:")
        self.x_field_label.pack(pady=(10, 5))
        self.x_field_entry = ctk.CTkEntry(self.sidebar_frame)
        self.x_field_entry.pack(pady=5, padx=10, fill="x")

        self.y_field_label = ctk.CTkLabel(self.sidebar_frame, text="Y Field:")
        self.y_field_label.pack(pady=(10, 5))
        self.y_field_entry = ctk.CTkEntry(self.sidebar_frame)
        self.y_field_entry.pack(pady=5, padx=10, fill="x")

        self.plot_button = ctk.CTkButton(self.sidebar_frame, text="Add XY Plot", command=self.add_xy_plot)
        self.plot_button.pack(pady=10, padx=10, fill="x")

        self.clear_button = ctk.CTkButton(self.sidebar_frame, text="Clear Plot", command=self.clear_plot)
        self.clear_button.pack(pady=5, padx=10, fill="x")

    def load_log_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("Binary Log", "*.bin"), ("All Files", "*.*")])
        if not file_path:
            return

        self.status_var.set("Loading log file...")
        self.update()

        try:
            self.schema = lp.load_schema()
            message_ids = sorted(self.schema.keys())
            message_names = [msg["name"] for msg in self.schema.values()]
            self.message_dropdown.configure(values=message_names)

            file_name = os.path.basename(file_path)
            self.current_file.set(file_name)
            self.current_log = file_path

            if message_ids:
                self.selected_message.set(self.schema[message_ids[0]]["name"])
                self.update_fields(self.schema[message_ids[0]]["name"])

            self.status_var.set(f"Loaded {len(self.frames)} message types from {file_name}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load log file: {str(e)}")
            self.status_var.set("Error loading file")

    def update_fields(self, selected_name):
        if selected_name not in [msg["name"] for msg in self.schema.values()]:
            return
        for msg_id, msg_def in self.schema.items():
            if msg_def["name"] == selected_name:
                field_names = list(msg_def["fields"])
                if field_names:
                    self.x_field_entry.delete(0, tk.END)
                    self.y_field_entry.delete(0, tk.END)
                    self.x_field_entry.insert(0, field_names[0])
                    if len(field_names) > 1:
                        self.y_field_entry.insert(0, field_names[1])
                break

    def add_xy_plot(self):
        msg_name = self.selected_message.get()
        x_field = self.x_field_entry.get()
        y_field = self.y_field_entry.get()

        if not all([msg_name, x_field, y_field]):
            print("Missing message or fields.")
            return

        # Find corresponding message ID
        msg_id = next((id for id, value in self.schema.items() if value["name"] == msg_name), None)
        if not msg_id:
            print("Message ID not found in schema.")
            return

        try:
            # Parse and cache if not already cached
            if msg_name not in self.frames:
                print(f"[i] Parsing log for message ID {msg_id}")
                frames, _ = lp.parse_log(self.current_log, [int(msg_id, 16)])
                self.frames[msg_name] = frames[int(msg_id, 16)]

            df = self.frames[msg_name]
            if x_field not in df or y_field not in df:
                print(f"[!] One or both fields not found in message '{msg_name}'.")
                return

            self.ax.plot(df[x_field], df[y_field], label=f"{msg_name}: {y_field} vs {x_field}")
            self.ax.legend()
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.canvas.draw()

        except Exception as e:
            print(f"[!] Error plotting fields: {e}")

    def clear_plot(self):
        self.ax.clear()
        self.canvas.draw()


def plot_selected_fields_matplotlib(frames, msg_id_str, fields, ax):
    """
    Plot selected fields using matplotlib, updating the provided axis.
    Supports automatic detection of 2D paths (pos_x, pos_y).
    """
    df = frames.get(msg_id_str)
    if df is None or df.empty:
        print(f"[i] No data found for {msg_id_str}")
        return

    has2d = {"pos_x", "pos_y"}.issubset(fields)
    has3d = {"pos_x", "pos_y", "pos_z"}.issubset(fields)

    if has3d:
        print(f"[i] 3D plot requested for {msg_id_str}, skipping (matplotlib 3D not integrated).")
        return

    if has2d:
        ax.plot(df["pos_x"], df["pos_y"], label=f"{msg_id_str} XY Path")
        ax.set_xlabel("pos_x")
        ax.set_ylabel("pos_y")
        return

    for field in fields:
        if field not in df.columns:
            print(f"[!] Field {field} missing in {msg_id_str}")
            continue
        ax.plot(df.index, df[field], label=f"{msg_id_str}: {field}")

    ax.legend()
    ax.set_title(f"Message {msg_id_str} Fields")

if __name__ == "__main__":
    app = SignalPlotterApp()
    app.mainloop()
