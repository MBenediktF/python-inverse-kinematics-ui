###############################################
# Main init file, Inverse Kinematics UI
# Version: 0.2
# Author: Benedikt Fassian
# Date: 06.01.2024
###############################################

from src.robotUI import RobotUI
import tkinter as tk

if __name__ == "__main__":
    app = RobotUI(tk.Tk())
    app.run()