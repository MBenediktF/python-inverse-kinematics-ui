###############################################
# UI Class for Inverse Kinematics
# Contains UI elements and Robotics toolbox functionality
# Version: 0.6
# Author: Benedikt Fassian
# Date: 06.01.2024
###############################################
# RoboticsToolbox Installation:
# pip install roboticstoolbox-python
# App Icon reference (Created by flaticon):
# https://www.flaticon.com/free-icons/robot
###############################################

import tkinter as tk
from tkinter import ttk
from tkinter import filedialog as fd
import roboticstoolbox as rtb
from scipy.spatial.transform import Rotation
import numpy as np
import csv
from tkinter.messagebox import showerror, showinfo
from helpers import parseInputString

class RobotUI:
    def __init__(self, master):

        self.master = master

        # Set application title and icon
        master.title("Inverse Kinematik Rechner by Benedikt Fassian")
        master.resizable(width=False, height=False)
        icon = tk.PhotoImage(file = "icon.png")
        master.iconphoto(False, icon)

        # Fonts
        bold_font = ('Helvetica', 14, 'bold')

        # Array to store result
        self.result = ["-", "-", "-", "-", "-", "-"]

        # Helper strings to show result strings
        self.subscript_numbers = "₀₁₂₃₄₅₆₇₈₉"
        self.coordinate_lables = ["X =", "Y =", "Z =", "A =", "B =", "C ="]
        self.coordinate_units = [" m", " m", " m", " rad", " rad", " rad"]

        # Presets from Roboticstoolbox
        self.preset_labels =    ["-", "Puma560", "UR3", "UR5", "UR10"]
        self.presets_dh = [False, rtb.models.DH.Puma560(), rtb.models.DH.UR3(), rtb.models.DH.UR5(), rtb.models.DH.UR10()]
        self.presets_urdf = [False, rtb.models.URDF.Puma560(), rtb.models.URDF.UR3(), rtb.models.URDF.UR5(), rtb.models.URDF.UR10()]


        ###############################################
        # Denavit-Hartenberg-Parameter Section
        ###############################################

        # Section title
        label_dh_params = tk.Label(master, text="_________________________      Denavit Hartenberg Parameter      __________________________", font=bold_font)
        label_dh_params.grid(row=1, column=0, columnspan=8, pady=10, sticky="s")

        # Table titles
        label_dh_params_gamma = ttk.Label(master, text="θ in rad")
        label_dh_params_gamma.grid(row=2, column=1, columnspan=1, pady=4, sticky="s")
        label_dh_params_d = ttk.Label(master, text="d in m")
        label_dh_params_d.grid(row=2, column=2, columnspan=1, pady=4, sticky="s")
        label_dh_params_a = ttk.Label(master, text="a in m")
        label_dh_params_a.grid(row=2, column=3, columnspan=1, pady=4, sticky="s")
        label_dh_params_alpha = ttk.Label(master, text="α in rad")
        label_dh_params_alpha.grid(row=2, column=4, columnspan=1, pady=4, sticky="s")
        label_dh_params_min = ttk.Label(master, text="Min")
        label_dh_params_min.grid(row=2, column=5, columnspan=1, pady=4, sticky="s")
        label_dh_params_max = ttk.Label(master, text="Max")
        label_dh_params_max.grid(row=2, column=6, columnspan=1, pady=4, sticky="s")
        label_dh_params_type = ttk.Label(master, text="Typ")
        label_dh_params_type.grid(row=2, column=7, columnspan=1, pady=4, sticky="s")

        # DH Params table
        self.entry_dh_params = []
        for i in range(6):
            joint_number = ttk.Label(master, width=8, text="   Gelenk "+str(i+1))
            entry_theta = ttk.Entry(master, width=8)
            entry_d = ttk.Entry(master, width=8)
            entry_a = ttk.Entry(master, width=8)
            entry_alpha = ttk.Entry(master, width=8)
            entry_min = ttk.Entry(master, width=8)
            entry_max = ttk.Entry(master, width=8)

            # Insert default values
            entry_theta.insert(0, "0")
            entry_d.insert(0, "0")
            entry_a.insert(0, "0")
            entry_alpha.insert(0, "0")
            entry_min.insert(0, "-pi")
            entry_max.insert(0, "pi")

            # Set Combobox values (first one can not be disabled)
            if(i==0):
                entry_type = ttk.Combobox(master, width=8, values=["Rotation", "Translation"], state="readonly")
                entry_type.set("Translation")
            else:
                entry_theta.configure(state="disabled")
                entry_d.configure(state="disabled")
                entry_a.configure(state="disabled")
                entry_alpha.configure(state="disabled")
                entry_max.configure(state="disabled")
                entry_min.configure(state="disabled")
                entry_type = ttk.Combobox(master, width=8, values=["Rotation", "Translation", "Deaktiviert"], state="readonly")
                entry_type.set("Deaktiviert")

            # Disable joints 2-6 by default
            if(i>1):
                entry_type.configure(state="disabled")

            # Resert Preset if a value is manually changed
            entry_theta.bind("<KeyRelease>", self.resetPreset)
            entry_d.bind("<KeyRelease>", self.resetPreset)
            entry_a.bind("<KeyRelease>", self.resetPreset)
            entry_alpha.bind("<KeyRelease>", self.resetPreset)
            entry_min.bind("<KeyRelease>", self.resetPreset)
            entry_max.bind("<KeyRelease>", self.resetPreset)

            # Place table elements
            joint_number.grid(row=i + 3, column=0, padx=10, pady=2, sticky="e")
            entry_theta.grid(row=i + 3, column=1, pady=2, sticky="w")
            entry_d.grid(row=i + 3, column=2, pady=2, sticky="w")
            entry_a.grid(row=i + 3, column=3, pady=2, sticky="w")
            entry_alpha.grid(row=i + 3, column=4, pady=2, sticky="w")
            entry_min.grid(row=i + 3, column=5, pady=2, sticky="w")
            entry_max.grid(row=i + 3, column=6, pady=2, sticky="w")
            entry_type.grid(row=i + 3, column=7, padx=10, pady=2, sticky="w")

            # Add inputs to array
            self.entry_dh_params.append((entry_theta, entry_d, entry_a, entry_alpha, entry_min, entry_max, entry_type))

            # Handle combobox events to disable and enable rowa
            self.entry_dh_params[i][6].bind("<<ComboboxSelected>>", lambda event, i=i: self.comboBoxChanged(i))

        # Select preset
        label_load_preset = ttk.Label(master, text="Preset:  ")
        label_load_preset.grid(row=10, column=1, columnspan=1, padx=0, pady=22, sticky="w")

        self.entry_load = ttk.Combobox(master, width=10, values=self.preset_labels, state="readonly")
        self.entry_load.set("-")
        self.entry_load.grid(row=10, column=1, columnspan=2, pady=2, sticky="e")
        self.entry_load.bind("<<ComboboxSelected>>", self.loadModelFromPreset)

        # Import DH parameters
        label_save = ttk.Label(master, text="Laden:  ")
        label_save.grid(row=10, column=3, columnspan=1, padx=0, pady=2, sticky="e")

        button_load = ttk.Button(master, width=5, text="Import", command=self.loadModelFromFile)
        button_load.grid(row=10, column=4, columnspan=2, padx=0, pady=2, sticky="w")

        # Export DH parameters
        label_save = ttk.Label(master, text="Sichern:  ")
        label_save.grid(row=10, column=5, columnspan=1, padx=0, pady=2, sticky="e")

        button_save = ttk.Button(master, width=5, text="Export", command=self.saveModel)
        button_save.grid(row=10, column=6, columnspan=2, padx=0, pady=2, sticky="w")

        # Plot and visualize buttons
        button_plot_kinematik = ttk.Button(master, width=16, text="Kinematik Plotten", command=self.plotRobot)
        button_plot_kinematik.grid(row=11, column=0, columnspan=4, padx=20, pady=0, sticky="e")

        button_visualize_kinematik = ttk.Button(master, width=16, text="Kinematik Visualisieren", command=self.visualizeRobot)
        button_visualize_kinematik.grid(row=11, column=4, columnspan=4, padx=20, pady=0, sticky="w")

        # Distance row (layout)
        mid_dist = ttk.Label(master)
        mid_dist.grid(row=12, column=0, columnspan=6, padx=0, pady=0, sticky="s")


        ###############################################
        # Inverse Kinematics Section
        ###############################################

        # Section title
        label_inverse_kinematik = ttk.Label(master, text="______________________________      Inverse Kinematik      _______________________________", font=bold_font)
        label_inverse_kinematik.grid(row=13, column=0, columnspan=8, pady=5, sticky="s")

        # Start position input
        label_start_position = ttk.Label(master, text="Start")
        label_start_position.grid(row=14, column=0, columnspan=3, padx=0, pady=2, sticky="s")

        # Type selector
        label_format_start = ttk.Label(master, text="Format: ")
        label_format_start.grid(row=15, column=0, columnspan=1, padx=0, pady=5, sticky="e")
        self.format_start = ttk.Combobox(master, width=10, values=["Gelenkposition", "Koordinaten"], state="readonly")
        self.format_start.set("Gelenkposition")
        self.format_start.grid(row=15, column=1, columnspan=2, padx=5, pady=5, sticky="w")

        # Input fields
        self.start_position = []
        for i in range(6):
            label = ttk.Label(master)
            entry = ttk.Entry(master, width=8)
            unit = ttk.Label(master)
            entry.insert(0, 0)
            
            label.grid(row=i + 16, column=0, padx=5, pady=2, sticky="e")
            entry.grid(row=i + 16, column=1, columnspan=1, pady=2, sticky="w")
            unit.grid(row=i + 16, column=2, columnspan=1, pady=2, sticky="w")

            self.start_position.append((entry, unit, label))

        # Handle combobox changed event in order to change units
        self.format_start.bind("<<ComboboxSelected>>", lambda event: self.setStartUnit(self.format_start.get()))

        # Target position input
        label_target_position = ttk.Label(master, text="Ziel")
        label_target_position.grid(row=14, column=3, columnspan=3, padx=0, pady=2, sticky="s")

        # Input type selector
        label_format_target = ttk.Label(master, text="Format: ")
        label_format_target.grid(row=15, column=3, columnspan=1, padx=0, pady=5, sticky="e")
        self.format_target = ttk.Combobox(master, width=10, values=["Gelenkposition", "Koordinaten"], state="readonly")
        self.format_target.set("Koordinaten")
        self.format_target.grid(row=15, column=4, columnspan=2, padx=5, pady=5, sticky="w")

        # Input fields
        self.target_position = []
        for i in range(6):
            label = ttk.Label(master)
            entry = ttk.Entry(master, width=8)
            unit = ttk.Label(master)
            entry.insert(0, 0)

            label.grid(row=i + 16, column=3, columnspan=1, padx=5, pady=2, sticky="e")
            entry.grid(row=i + 16, column=4, columnspan=1, pady=2, sticky="w")
            unit.grid(row=i + 16, column=5, columnspan=1, pady=2, sticky="w")

            self.target_position.append((entry, unit, label))

        # Handle combobox changed event in order to change units
        self.format_target.bind("<<ComboboxSelected>>", lambda event: self.setTargetUnit(self.format_target.get()))

        # Solver
        label_solver = ttk.Label(master, text="Solver")
        label_solver.grid(row=14, column=6, columnspan=2, padx=0, pady=2, sticky="s")
        self.solver = ttk.Combobox(master, width=10, values=["IK_LM", "IK_GN", "IK_NR"], state="readonly")
        self.solver.set("IK_LM")
        self.solver.grid(row=15, column=6, columnspan=2, padx=5, pady=5, sticky="s")

        # Limits
        label_limits = ttk.Label(master, text="Limits")
        label_limits.grid(row=17, column=6, columnspan=2, padx=0, pady=2, sticky="s")
        self.limits = ttk.Combobox(master, width=10, values=["Aktiv", "Unbegrenzt"], state="readonly")
        self.limits.set("Aktiv")
        self.limits.grid(row=18, column=6, columnspan=2, padx=5, pady=5, sticky="s")

        # Calculate button
        label_calculate = ttk.Label(master, text="Berechnen")
        label_calculate.grid(row=20, column=6, columnspan=2, padx=0, pady=2, sticky="s")
        button_calculate = ttk.Button(master, width=8, text="Start", command=self.calculate)
        button_calculate.grid(row=21, column=6, columnspan=2, sticky="s")

        # Result
        title_result = ttk.Label(master, text="_________________________     Ergebnis     _________________________")
        title_result.grid(row=24, column=0, columnspan=8, pady=10, sticky="s")

        # Output text
        self.label_result = ttk.Label(master, anchor="center")
        self.label_result.grid(row=26, column=0, columnspan=8, padx=0, pady=5, sticky="s")

        # Plot and visualize result buttons
        button_plot_result = ttk.Button(master, width=16, text="Ergebnis Plotten", command=self.plotResult)
        button_plot_result.grid(row=28, column=0, columnspan=4, padx=20, pady=12, sticky="e")
        button_visualize_result = ttk.Button(master, width=16, text="Ergebnis Visualisieren", command=self.visualizeResult)
        button_visualize_result.grid(row=28, column=4, columnspan=4, padx=20, pady=12, sticky="w")

        # Distance element (layout)
        bottom_dist = ttk.Label(master, width=8, text="")
        bottom_dist.grid(row=30, column=0, columnspan=6, padx=0, pady=0, sticky="s")

        # Set units to defaults
        self.setStartUnit(self.format_start.get())
        self.setTargetUnit(self.format_target.get())
        self.createResultString(self.format_target.get())

    # Handles comboboxChanged events from DH table
    def comboBoxChanged(self, i):
        # Dsable or enable row
        if self.entry_dh_params[i][6].get() == "Deaktiviert":
            self.setRow(i, "disabled")
        else: 
            self.setRow(i, "normal")
        # Reset selected preset (DH value changed)
        self.resetPreset(False)
        # Update result string
        self.createResultString(self.format_target.get())

    # Disables or enables selected DH row and following ones
    def setRow(self, row, state):
        for i in range(6):
            # Set the state of the selected rows
            self.entry_dh_params[row][i].configure(state=state)
        if row<5:
            # Also enable the Combobox of the next row
            self.entry_dh_params[row+1][6].configure(state="readonly")
        # Also disable all joints > disabled joint
        if state == "disabled":
            for i in range(5-row):
                self.entry_dh_params[5-i][6].set("Deaktiviert")
                for j in range(7):
                    self.entry_dh_params[5-i][j].configure(state="disabled")
        # Update start and target position inputs
        self.setStartUnit(self.format_start.get())
        self.setTargetUnit(self.format_target.get())

    # Resets the selected preset
    def resetPreset(self, event=False): 
        self.entry_load.set("-")
        self.result = ["-", "-", "-", "-", "-", "-"]

    # Load preset
    def loadModelFromPreset(self, event=False): self.loadModel(False)

    # Load model from file
    def loadModelFromFile(self, event=False): self.loadModel(True)

    # Load a robot from preset or from file
    def loadModel(self, fromFile):
        # Reset Joints 2-6 (DH-Table)
        self.entry_dh_params[1][6].set("Deaktiviert")
        self.setRow(1, "disabled")

        if fromFile:
            # Load Robot from file
            file_path = fd.askopenfilename(defaultextension=".csv", filetypes=[("CSV-Dateien", "*.csv")])
            self.resetPreset()
            try:
                # CSV-Datei lesen
                with open(file_path, mode='r') as file:
                    reader = csv.reader(file)
                    data = list(reader)

                    # Check csv file structure
                    if len(data) >= 2 and len(data[0]) == 8:
                        # Read rows
                        for i in range(len(data)-1):
                            if data[i+1][7] in ["Rotation", "Translation"]:
                                self.entry_dh_params[i][6].set(data[i+1][7])  # Gelenktyp
                                self.setRow(i, "normal")
                            else:
                                break
                            self.entry_dh_params[i][0].delete(0, 'end')
                            self.entry_dh_params[i][0].insert(0, data[i+1][1])  # θ in rad
                            self.entry_dh_params[i][1].delete(0, 'end')
                            self.entry_dh_params[i][1].insert(0, data[i+1][2])  # d in m
                            self.entry_dh_params[i][2].delete(0, 'end')
                            self.entry_dh_params[i][2].insert(0, data[i+1][3])  # a in m
                            self.entry_dh_params[i][3].delete(0, 'end')
                            self.entry_dh_params[i][3].insert(0, data[i+1][4])  # alpha in m
                            self.entry_dh_params[i][4].delete(0, 'end')
                            self.entry_dh_params[i][4].insert(0, data[i+1][5])  # alpha in m
                            self.entry_dh_params[i][5].delete(0, 'end')
                            self.entry_dh_params[i][5].insert(0, data[i+1][6])  # alpha in m
                            self.entry_dh_params[i][6].delete(0, 'end')
                    else: 
                        showerror(message="Ungültiges Dateiformat für Denavit-Hartenberg-Parameter.")
            except Exception as e:
                showerror(message=f"Fehler beim Laden der Denavit-Hartenberg-Parameter: {str(e)}")
                return
            
        else:
            # Load robot from preset
            if self.entry_load.get() == "-": return
            robot = self.presets_dh[self.preset_labels.index(self.entry_load.get())]
            for i, link in enumerate(robot.links):
                # Get joint type
                if link.isrevolute:
                    self.entry_dh_params[i][6].set("Rotation")
                else:
                    self.entry_dh_params[i][6].set("Translation")
                self.setRow(i, "normal")
                # Get Dh Params
                self.entry_dh_params[i][0].delete(0, 'end')
                self.entry_dh_params[i][0].insert(0, robot.links[i].theta)
                self.entry_dh_params[i][1].delete(0, 'end')
                self.entry_dh_params[i][1].insert(0, robot.links[i].d)
                self.entry_dh_params[i][2].delete(0, 'end')
                self.entry_dh_params[i][2].insert(0, robot.links[i].a)
                self.entry_dh_params[i][3].delete(0, 'end')
                self.entry_dh_params[i][3].insert(0, robot.links[i].alpha)
                self.entry_dh_params[i][4].delete(0, 'end')
                self.entry_dh_params[i][5].delete(0, 'end')
                # Set qlim (to default if not specified in robot object)
                if robot.links[i].qlim is None:
                    # Set to default
                    self.entry_dh_params[i][4].insert(0, "-pi")
                    self.entry_dh_params[i][5].insert(0, "+pi")
                else:
                    # Get limits from robot object
                    self.entry_dh_params[i][4].insert(0, robot.links[i].qlim[0])
                    self.entry_dh_params[i][5].insert(0, robot.links[i].qlim[1])

        # Update all units
        self.setStartUnit(self.format_start.get())
        self.setTargetUnit(self.format_target.get())
        self.createResultString(self.format_target.get())
        return
    
    # Save the dh table to a .csv file
    def saveModel(self):
        try:
            # Get the file path from dialog
            file_path = fd.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV-Dateien", "*.csv")])
            data = [
                ['Gelenk', 'θ in rad', 'd in m', 'a in m', 'alpha in m', 'Min', 'Max', 'Gelenktyp']
            ]

            # Add data for each joint
            for i in range(6):
                if self.entry_dh_params[i][6].get() == "Deaktiviert": break
                data.append([str(i+1), self.entry_dh_params[i][0].get(), self.entry_dh_params[i][1].get(), self.entry_dh_params[i][2].get(), self.entry_dh_params[i][3].get(), self.entry_dh_params[i][4].get(), self.entry_dh_params[i][5].get(), self.entry_dh_params[i][6].get()])

            # Write to csv file
            with open(file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(data)
            showinfo(message="Speichern erfolgreich!")
        except Exception as e:
            showerror(message=f"Fehler beim Speichern der Denavit-Hartenberg-Parameter: {str(e)}")
            return
        return
    
    # Plot robot (from DH position)
    def plotRobot(self): self.showRobot(visualize=False, result=False)

    # Visualize Robot (from DH position)
    def visualizeRobot(self): self.showRobot(visualize=True, result=False)
    
    # Plot robot (with result position)
    def plotResult(self): self.showRobot(visualize=False, result=True)

    # Visualize robot (with result position)
    def visualizeResult(self): self.showRobot(visualize=True, result=True)

    # Robot plot
    def showRobot(self, visualize=False, result=False):
        q = []
        for i in range(6):
            if result:
                # Get the result joint values
                if self.result[i] == "-": break
                q.append(float(self.result[i]))
            else:
                # Get position data from Denavit Hartenberg Parameters
                try:
                    if self.entry_dh_params[i][6].get() == "Rotation":
                        q.append(parseInputString(self.entry_dh_params[i][0].get()))
                    elif self.entry_dh_params[i][6].get() == "Translation":
                        q.append(parseInputString(self.entry_dh_params[i][1].get()))
                    else:
                        break
                except:
                    showerror(message="Eingabefehler. Die Denavit-Hartenberg-Parameter liegen nicht im richtigen Format vor.")
                    return False
        try:
            if visualize:
                # Plot robot from preset
                if self.entry_load.get() == "-": 
                    showerror(message="Kein Preset gewählt. Individuelle Eingaben können nicht visualisiert werden.")
                    return
                robot = self.presets_urdf[self.preset_labels.index(self.entry_load.get())]
                robot.plot(q=q)
            else:
                # Plot Dh robot
                if self.entry_load.get() == "-":
                    # Create robot object from given DH parameters
                    robot = self.createRobotFromDH()
                    if not robot: return
                else:
                    # Get robot data from preset
                    robot = self.presets_dh[self.preset_labels.index(self.entry_load.get())]
                robot.plot(q=q)
        except:
            showerror(message="Darstellung nicht möglich. Eingaben und Ergebnis prüfen!")
        return

    # Updates the start position input units       
    def setStartUnit(self, unit):
        # Set lables if rotation input
        if unit == "Gelenkposition":
            for i in range(6):
                # Set Lables for rotation
                if self.entry_dh_params[i][6].get()=="Rotation":
                    self.start_position[i][1].config(text=" rad")
                    self.start_position[i][2].config(text="θ"+self.subscript_numbers[i+1]+ " =")
                # Set lables for translation
                else:
                    self.start_position[i][1].config(text=" m")
                    self.start_position[i][2].config(text="d"+self.subscript_numbers[i+1]+ " =")
                # Disable entry if joint disabled
                if self.entry_dh_params[i][6].get()=="Deaktiviert":
                    self.start_position[i][0].configure(state="disabled")
                else:
                    self.start_position[i][0].configure(state="normal")
        # Set lables if translation input
        else:
            for i in range(6):
                self.start_position[i][0].config(state="normal")
                self.start_position[i][1].config(text=self.coordinate_units[i])
                self.start_position[i][2].config(text=self.coordinate_lables[i])

    # Updates the target position inputs
    def setTargetUnit(self, unit):
        # Set lables if rotation input
        if unit == "Gelenkposition":
            for i in range(6):
                # Set Lables for rotation
                if self.entry_dh_params[i][6].get()=="Rotation":
                    self.target_position[i][1].config(text="rad")
                    self.target_position[i][2].config(text="θ"+self.subscript_numbers[i+1]+ " =")
                # Set lables for translation
                else:
                    self.target_position[i][1].config(text="m")
                    self.target_position[i][2].config(text="d"+self.subscript_numbers[i+1]+ " =")
                # Disable entry if joint disabled
                if self.entry_dh_params[i][6].get()=="Deaktiviert":
                    self.target_position[i][0].configure(state="disabled")
                else:
                    self.target_position[i][0].configure(state="normal")
        # Set lables if translation input
        else:
            for i in range(6):
                self.target_position[i][0].config(state="normal")
                self.target_position[i][1].config(text=self.coordinate_units[i])
                self.target_position[i][2].config(text=self.coordinate_lables[i])

    # Creates result output string
    def createResultString(self, unit):
        result_text=""
        # Set lables if koordinate input, that means rotational output
        if unit == "Koordinaten":
            for i in range(6):
                # Set Lables for rotation
                if self.entry_dh_params[i][6].get()=="Rotation":
                    result_text += " θ"+self.subscript_numbers[i+1]+ "= " + self.result[i] + " rad, "
                # Set lables for translation
                elif self.entry_dh_params[i][6].get()=="Translation":
                    result_text += " d"+self.subscript_numbers[i+1]+ "= " + self.result[i] + " m, "
                else:
                    break
        # Set lables if translation input
        else:
            for i in range(6):
                result_text += self.coordinate_lables[i] + "= " + self.result[i] + self.coordinate_units[i] + ", "
        self.label_result.config(text=result_text[:-2])

    # Creates a robot object from the DH-table input
    def createRobotFromDH(self):
        robot_config = []
        # Read robot conig for each joint
        for i in range(6):
            if self.entry_dh_params[i][6].get() == "Deaktiviert": break
            try:
                theta = parseInputString(self.entry_dh_params[i][0].get())
                d = parseInputString(self.entry_dh_params[i][1].get())
                a = parseInputString(self.entry_dh_params[i][2].get())
                alpha = parseInputString(self.entry_dh_params[i][3].get()) 
                min = parseInputString(self.entry_dh_params[i][4].get()) 
                max = parseInputString(self.entry_dh_params[i][5].get()) 
            except Exception as e:
                print(e)
                showerror(message="Eingabefehler. Die Denavit-Hartenberg-Parameter liegen nicht im richtigen Format vor.")
                return False
            # Add joint to robot config (as object)
            if self.entry_dh_params[i][6].get() == "Rotation":
                robot_config.append(rtb.RevoluteDH(d=d, a=a, alpha=alpha, qlim=[min, max]))
            elif self.entry_dh_params[i][6].get() == "Translation":
                robot_config.append(rtb.PrismaticDH(theta=theta, a=a, alpha=alpha, qlim=[min, max]))
            else:
                showerror(message="Eingabefehler. Der gewählte Joint Typ ist nicht bekannt.")
                return False
        # Create and return robot object
        return rtb.DHRobot(robot_config, name="Robot")

    # Calculate the result from the given input
    def calculate(self):    
        # Get start position input
        if(self.format_start.get()=="Gelenkposition"):
            q_start = []
            for i in range(6):
                if self.entry_dh_params[i][6].get() in ["Rotation", "Translation"]:
                    q_start.append(parseInputString(self.start_position[i][0].get()))
                else:
                    break
        else:
            # Right now only joint position input is supported
            showerror(message="Funktion icht verfügbar. Bisher können nur Gelenkpositionen als Start genutzt werden.")
            return
        
        # Get target position input
        target_transformation = np.empty((4, 4))
        if(self.format_target.get()=="Koordinaten"):
            X = parseInputString(self.target_position[0][0].get())
            Y = parseInputString(self.target_position[1][0].get())
            Z = parseInputString(self.target_position[2][0].get())
            A = parseInputString(self.target_position[3][0].get())
            B = parseInputString(self.target_position[4][0].get())
            C = parseInputString(self.target_position[5][0].get())
            rotation = Rotation.from_euler('XYZ', [A, B, C])
            target_transformation = np.vstack((np.hstack((rotation.as_matrix(), [[X], [Y], [Z]])), [0, 0, 0, 1]))
        else:
            # Right now only coordinate input is supported
            showerror(message="Funktion nicht verfügbar. Bisher können nur Koordinaten als Ziel genutzt werden.") 
            return

        # Create rtb robot model
        robot = self.createRobotFromDH()
        if not robot: return

        # Calculate Inverse Kinematics
        # Levemberg-Marquadt selected
        if self.solver.get() == "IK_LM":
            result = robot.ikine_LM(target_transformation, q0=q_start, joint_limits=(self.limits.get()=="Aktiv"))
        # Quadratic programming approach solver selected (not available right now)
        elif self.solver.get() == "IK_QP":
            #result = robot.ikine_QP(target_transformation, q0=q_start, joint_limits=(self.limits.get()=="Aktiv"))
            print("Solver not available roght now")
        #  Gauss-Newton selected
        elif self.solver.get() == "IK_GN":
            result = robot.ikine_GN(target_transformation, q0=q_start, joint_limits=(self.limits.get()=="Aktiv"))
        # Newton-Raphson selected
        elif self.solver.get() == "IK_NR":
            result = robot.ikine_NR(target_transformation, q0=q_start, joint_limits=(self.limits.get()=="Aktiv"))
        # Wrong selection error
        else:
            showerror(message="Der gewählte Solver steht nicht zur Verfügung.") 
            return
        # Print result (internal)
        print(result)

        # Handle "no solution found"
        if not result.success: 
            showerror(message="Mit dem gewählten Solver konnte keine Lösung gefunden werden, um die Zielposition mit der gegebenen Kinematik zu erreichen.")
            return 
        # Solution found
        else:
            for i in range(6):
                if self.entry_dh_params[i][6].get() == "Deaktiviert":
                    self.result[i] = "-"
                else:
                    self.result[i] = str(round(result.q[i], 4))
            # Output result
            self.createResultString(self.format_target.get())

    # Start UI
    def run(self):
        self.master.mainloop()