# Apriltag setup 
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
import os

# -----------------------------------------------------------------------------
# File configuration
# -----------------------------------------------------------------------------
parentFolder = 'Implementation\\Host_24-04-23\\'
sourceRef = 'Source_reference_calibration_C'
robotRef = 'Robot_reference_calibration_C'

# -----------------------------------------------------------------------------
# Main script
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    refVideo = sourceRef
    refType = 0 # 0 = source elevation, 1 = robot elevation
    completeVideo = parentFolder + refVideo + '.mp4'

    # Execute matrix transformation script to update config file
    os.system(f'python {parentFolder}setup_matrices.py --type {refType} --video {completeVideo}')

    if refType == 0:
        # Execute coordinate setup script to update config file
        os.system(f'python {parentFolder}setup_source_borders.py --video {completeVideo}')

    refVideo = robotRef
    refType = 1 # 0 = source elevation, 1 = robot elevation
    completeVideo = parentFolder + refVideo + '.mp4'

    # Execute matrix transformation script to update config file
    os.system(f'python {parentFolder}setup_matrices.py --type {refType} --video {completeVideo}')