###############################################
# Helper funktions for inverse kinematic ui
# Version: 0.1
# Author: Benedikt Fassian
# Date: 04.01.2024
###############################################

from math import pi

# Converts string input or equation to float
# INPUTS: String to parse <string>
# OUTPUTS: Result <float>
def parseInputString(value):
    # Set empty value to "0"
    if value=="": value="0"
    # Replace , with .
    value = value.replace(",", ".")
    # Solve equations
    try:
        value = eval(value)
    except: 
        raise
    # Convert value to float
    value = float(value)
    return value