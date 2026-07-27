# Aeon Feeder

![GitHub release](https://img.shields.io/github/v/release/SainsburyWellcomeCentre/aeon_feeder.svg)

Complete project repository for the underground feeder used in project Aeon.

## Build your own

Check out the **[latest release](https://github.com/SainsburyWellcomeCentre/aeon_feeder/releases/latest)** for production Gerbers, BOM, and assembly drawings to build your own beambreak filter.

## Folder Structure

### Firmware

This folder contains the native code for the Raspberry Pi Pico

### eCAD

This folder contains 2 Altium Designer project folders for the Application Board and Beambreak Sensor.

### mCAD

This folder contains the complete mechanical design in Inventor 2023 project format.

To use this project within Inventor 2023+, the user should first open the Inventor Project file called **AEON.Feeder.ipj**

This will ensure Inventor is using the correct Project file to correctly find and assemble the the parts and assemblies within the Feeder.

The project can be opened from any location where the user has chosen to pull the whole project.

The main assembly for the complete model is **AEON.Feeder.\_Main.iam**

## 💻 Software Requirements

- Microsoft Visual Studio and Raspberry Pi Pico SDK as setup within the [Getting Started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)

To view or edit the source design files:

- **Altium Designer 20.0+** _(electronic design)_ — Academic licenses via [Altium Education](https://www.altium.com/education)
- **Inventor 2023+** _(mechanical design)_ — Academic licenses via [Autodesk Education](https://www.autodesk.com/education/free-software/inventor)

## 📜 License

**Sainsbury Wellcome Centre hardware is released under** [Creative Commons Attribution-ShareAlike 4.0 International](http://creativecommons.org/licenses/by-sa/4.0/).

You are free to:

- **Share** — copy and redistribute the material in any medium or format
- **Adapt** — remix, transform, and build upon the material for any purpose

Under the following terms:

- **Attribution** — Give appropriate credit, link to the license, and indicate changes.
- **ShareAlike** — Distribute your contributions under the same license.
- **No additional restrictions** — Don't apply legal or technological measures that prevent others from doing anything the license permits.

> For the full legal text, see [LICENSE](LICENSE).

## Citation Policy

If you use this software or hardware, please cite it as below:

D. Campagner, J. Bhagat, G. Lopes, L. Calcaterra, A. G. Pouget, A. Almeida, T. T. Nguyen, C. H. Lo, T. Ryan, B. Cruz, F. J. Carvalho, Z. Li, A. Erskine, J. Rapela, O. Folsz, M. Marin, J. Ahn, S. Nierwetberg, S. C. Lenzi, J. D. S. Reggiani, SGEN group – SWC GCNU Experimental Neuroethology Group. _Aeon: an open-source platform to study the neural basis of ethological behaviours over naturalistic timescales._ Preprint at https://doi.org/10.1101/2025.07.31.664513 (2025)

[![DOI:10.1101/2025.07.31.664513](https://img.shields.io/badge/DOI-10.1101%2F2025.07.31.664513-AE363B.svg)](https://doi.org/10.1101/2025.07.31.664513)

## 📧 Contact

- **Author**: [Sainsbury Wellcome Centre FabLabs](https://www.sainsburywellcome.org/content/fablab)
- **Email**: [swc.fablabs@ucl.ac.uk](mailto:swc.fablabs@ucl.ac.uk)
- **Website**: [FabLabs Documentation](https://sainsburywellcomecentre.github.io/fablabs-documentation/aeon_feeder)
