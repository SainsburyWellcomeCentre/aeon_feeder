# Aeon Feeder
Complete project repository for the underground feeder used in project Aeon.
## Folder Structure
### Code
This folder contains the VS Code project and code for the Raspberry Pi Pico
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

D. Campagner, J. Bhagat, G. Lopes, L. Calcaterra, A. G. Pouget, A. Almeida, T. T. Nguyen, C. H. Lo, T. Ryan, B. Cruz, F. J. Carvalho, Z. Li, A. Erskine, J. Rapela, O. Folsz, M. Marin, J. Ahn, S. Nierwetberg, S. C. Lenzi, J. D. S. Reggiani, SGEN group&mdash;SWC GCNU Experimental Neuroethology Group. _Aeon: an open-source platform to study the neural basis of ethological behaviours over naturalistic timescales._ Preprint at https://doi.org/10.1101/2025.07.31.664513 (2025)

[![DOI:10.1101/2025.07.31.664513](https://img.shields.io/badge/DOI-10.1101%2F2025.07.31.664513-AE363B.svg)](https://doi.org/10.1101/2025.07.31.664513)

