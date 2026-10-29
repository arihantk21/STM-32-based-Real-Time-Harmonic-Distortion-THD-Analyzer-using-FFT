\# STM32-Based Real-Time Harmonic Distortion (THD) Analyzer Using FFT



A microcontroller-based project for analyzing signal harmonics and Total Harmonic Distortion (THD) using an STM32 microcontroller and Fast Fourier Transform (FFT).



\## Project Overview



This project uses an STM32F411CEU6 microcontroller to acquire signal samples, process them for frequency-domain analysis, and communicate measurement data to a PC for visualization.



\## Hardware and Software



\- \*\*Microcontroller:\*\* STM32F411CEU6 (ARM Cortex-M4F)

\- \*\*Firmware IDE:\*\* STM32CubeIDE

\- \*\*Programming language:\*\* C

\- \*\*PC visualization:\*\* Python

\- \*\*Communication:\*\* USB CDC

\- \*\*Signal processing:\*\* Fast Fourier Transform (FFT)



\## Repository Structure



```text

THD\_Analyzer\_GitHub/

├── Firmware/

│   └── fft\_analyzer/

├── Python/

│   └── spectrum\_plot.py

├── Documentation/

└── README.md

```



\## Features



\- ADC-based signal acquisition

\- FFT-based frequency analysis

\- USB CDC communication with a computer

\- Python-based spectrum visualization



Refer to the firmware source code for the implemented functionality and configuration.



\## Getting Started



\### Firmware



1\. Install STM32CubeIDE.

2\. Import the `Firmware/fft\_analyzer` project into the workspace.

3\. Review the project configuration and target microcontroller.

4\. Build the project and program the compatible STM32 hardware.



\### Python Visualization



1\. Install Python 3.

2\. Install the Python packages required by `Python/spectrum\_plot.py`.

3\. Connect the device and configure the appropriate communication port as required by the script.

4\. Run the visualization script.



\## Notes



\- Hardware connections and signal input requirements must be checked before testing.

\- ADC configuration, sampling parameters, FFT implementation, and THD calculation should be verified against the firmware source code.

\- Generated build files are excluded from version control.



\## License



A license has not yet been specified.



