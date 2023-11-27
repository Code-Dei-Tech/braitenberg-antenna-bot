# Robot sensor symmetry calibration
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvConfiguration as cfg

# Import system libraries
import time
import os 
import numpy as np
import pandas as pd
import argparse
from scipy.optimize import curve_fit

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
# Parse arguments
def parseArguments():
    # Parse the arguments
    parser = argparse.ArgumentParser(description='Calibrate the sensor symmetry.')
    parser.add_argument('-n', '--name', type=str, default="run", help='Name of the run')
    args = parser.parse_args()

    # Return the arguments
    return args.name

# -----------------------------------------------------------------------------
# Main function
# -----------------------------------------------------------------------------
def main(name: str = "run"):
    # Load the data
    symmetryFolder = cfg.sensorSymmetryFolder
    configPath = os.path.dirname(__file__) + "/" + "hvConfiguration.py"
    refCol = "sensor"+str(cfg.symRef)
    minuCol = "sensor"+str(cfg.symMinu)
    subtCol = "sensor"+str(cfg.symSubt)

    # Initialize global dataframe
    df = pd.DataFrame()

    # Loop through all files saved in the symmetry folder with a run name
    for file in os.listdir(symmetryFolder):
        if name in file:
            # Read csv
            dfTemp = pd.read_csv(symmetryFolder + file)

            # Append to global dataframe
            df = pd.concat([df, dfTemp], ignore_index=True)

    # If dataframe is empty
    if df.empty:
        raise Exception("No data found for run name: " + name)

    # Create a new column for the sensor difference
    df["sensorDiff"] = df[minuCol] - df[subtCol]

    # Sort by the middle sensor
    df = df.sort_values(by=[refCol])

    # Cut off rows for outlier middle sensor values above 20000
    df = df[df[refCol] < 20000]

    # Fit a straight line curve to the difference column
    # Get the coefficients
    # coeffs = np.polyfit(df[refCol], df["sensorDiff"], 1)
    coeffs = curve_fit(cfg.func0, df[refCol], df["sensorDiff"])[0]

    # Update the coefficients parameter in the configuration file
    # Open the configuration py file
    with open(configPath, 'r') as file:
        filedata = file.read()

    # Replace the line with the new one
    newCoeffs = "symCoeffs = " + str(list(coeffs))

    for line in filedata.splitlines():
        if "symCoeffs" in line:
            filedata = filedata.replace(line, newCoeffs)

    # Write the file out again
    with open(configPath, 'w') as file:
        file.write(filedata)

    # Print the coefficients
    print("Symmetry coefficients: " + str(coeffs))

if __name__ == "__main__":
    main(parseArguments())