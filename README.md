# Aeon Feeder
Complete project repository for the underground feeder used in project Aeon.
## Folder Structure
### Code
This folder contains the VS Code project and code for the Raspberry Pi Pico
### Firmware
This folder contains a compiled binary file in uf2 format which can be directly uploaded to the Pi Pico. See Notes
### eCAD
This folder contains 2 Altium Designer project folders for the Application Board and Beam Brake.

Within each project folder all the design files as well as another folder called __Project Outputs__

The __Project Outputs__ folder contains files for Documentation, Assembly and Fabrication.
### mCAD
This folder contains the complete mechanical design in Inventor 2023 project format.

To use this project within Inventor 2023+, the user should first open the Inventor Project file called **AEON.Feeder.ipj**

This will ensure Inventor is using the correct Project file to correctly find and assemble the the parts and assemblies within the Feeder.

The project can be opened from any location where the user has chosen to pull the whole project.

The main assembly for the complete model is **AEON.Feeder._Main.iam**

## Notes
The required software for the project is as follows
* Microsoft Visual Studio and Raspberry Pi Pico SDK as setup within the [Getting Started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
* Altium Designer 23.5 or newer. Academic licenses can be obtained by contacting [Altium Education](https://www.altium.com/education/)
* Inventor Pro 2023 or newer. Academic licenses can be obtained by contacting [Autodesk Education](https://www.autodesk.com/education/home)

## Citation Policy

If you use this software or hardware, please cite it as below:

Sainsbury Wellcome Centre Foraging Behaviour Working Group. (2023). Aeon: An open-source platform to study the neural basis of ethological behaviours over naturalistic timescales,  https://doi.org/10.5281/zenodo.8413142

[![DOI](https://zenodo.org/badge/485512362.svg)](https://zenodo.org/badge/latestdoi/485512362)
