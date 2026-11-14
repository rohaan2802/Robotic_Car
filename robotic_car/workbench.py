"""Offline desktop workbench for inspecting grids and trying range observations."""

import math
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Patch

from .__main__ import SAVED_MAP, demo_map
from .mapping import MAX_RANGE_CM, OccupancyMap, Pose


class Workbench(ttk.Frame):
    def __init__(self, window):
        super().__init__(window, padding=16)
        self.pack(fill="both", expand=True)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)
        self.grid_map = demo_map()
        self.last_hit = None
        self.source = "Synthetic demo"
        self.status = tk.StringVar()

        ttk.Label(self, text="Robotic Car · Mapping Workbench",
                  font=("Segoe UI", 20, "bold")).grid(sticky="w")
        ttk.Label(self, text="Offline map inspection and manual range observations").grid(sticky="w")

        toolbar = ttk.Frame(self)
        toolbar.grid(row=2, sticky="ew", pady=10)
        for label, action in (
            ("Synthetic demo", lambda: self.set_map(demo_map(), "Synthetic demo")),
            ("Original saved map", lambda: self.open_map(SAVED_MAP, "Original saved map")),
            ("Blank grid", lambda: self.set_map(OccupancyMap(120), "Blank grid")),
            ("Open .npy", self.choose_map), ("Export .npy", self.export),
        ):
            ttk.Button(toolbar, text=label, command=action).pack(side="left", padx=(0, 7))

        inputs = ttk.Frame(self)
        inputs.grid(row=3, sticky="ew", pady=(0, 10))
        self.values = []
        for column, (name, default) in enumerate(zip(
                ("X (cm)", "Y (cm)", "Heading (°)", "Range (cm)"), (0, 0, 45, 22))):
            ttk.Label(inputs, text=name).grid(row=0, column=column, sticky="w")
            value = tk.StringVar(value=str(default))
            ttk.Entry(inputs, textvariable=value, width=12).grid(row=1, column=column, padx=(0, 8))
            self.values.append(value)
        ttk.Button(inputs, text="Apply observation", command=self.apply).grid(row=1, column=4)

        fig = Figure(figsize=(8, 5), layout="constrained", facecolor="#f7f4ee")
        self.axis = fig.add_subplot()
        self.canvas = FigureCanvasTkAgg(fig, master=self)
        self.canvas.get_tk_widget().grid(row=4, sticky="nsew")
        ttk.Label(self, textvariable=self.status, wraplength=710).grid(row=5, sticky="w", pady=(8, 0))
        self.refresh()

    def set_map(self, grid, source):
        self.grid_map, self.source = grid, source
        self.last_hit = None
        self.refresh()

    def open_map(self, path, label):
        try:
            self.set_map(OccupancyMap.load(path), label)
        except (OSError, ValueError) as error:
            messagebox.showerror("Could not open the map", str(error), parent=self)

    def choose_map(self):
        path = filedialog.askopenfilename(filetypes=[("NumPy grids", "*.npy")], parent=self)
        if path:
            self.open_map(path, "Imported map")

    def export(self):
        path = filedialog.asksaveasfilename(defaultextension=".npy",
                                           filetypes=[("NumPy grids", "*.npy")], parent=self)
        if path:
            try:
                np.save(path, self.grid_map.grid)
                self.status.set("Map exported.")
            except OSError as error:
                messagebox.showerror("Could not export the map", str(error), parent=self)

    def apply(self):
        try:
            x, y, heading, distance = (float(value.get()) for value in self.values)
            if not all(map(math.isfinite, (x, y, heading, distance))):
                raise ValueError("Enter finite numbers for pose and range.")
            row, col = self.grid_map.cell(x, y)
            size = self.grid_map.grid.shape[0]
            if not (0 <= row < size and 0 <= col < size):
                raise ValueError("The observation must start inside the grid.")
            if not self.grid_map.update(Pose(x, y, heading), distance):
                raise ValueError(f"Enter a range greater than zero and at most {MAX_RANGE_CM} cm.")
        except ValueError as error:
            self.status.set(str(error))
            return
        self.last_hit = (x, y, heading, distance)
        self.refresh()

    def refresh(self):
        self.axis.clear()
        self.grid_map.draw(self.axis, crop=True)
        self.axis.set_title(self.source)
        colors = ("#f4a444", "#142537", "#40c9b8")
        handles = [Patch(color=color, label=label) for color, label in
                   zip(colors, ("Free", "Unknown", "Occupied"))]
        if self.last_hit:
            x, y, heading, distance = self.last_hit
            angle = math.radians(heading)
            ray, = self.axis.plot([x, x + distance * math.cos(angle)],
                                  [y, y + distance * math.sin(angle)],
                                  color="#d34645", linestyle="--", label="Latest observation")
            handles.append(ray)
        self.axis.legend(handles=handles, loc="upper right", fontsize=8)
        free, unknown, occupied = self.grid_map.counts()
        self.status.set(f"Free {free:,}   ·   Unknown {unknown:,}   ·   Occupied {occupied:,}")
        self.canvas.draw_idle()


def main():
    window = tk.Tk()
    window.title("Robotic Car Mapping Workbench")
    window.geometry("900x690")
    window.minsize(720, 570)
    Workbench(window)
    window.mainloop()


if __name__ == "__main__":
    main()
