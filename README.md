## The Design of a Salinity Sensor for Antarctic Research
EEE4022S_Salinity_JCP24-03
 
This project focused on the design of a salinity sensor for measuring the salinity of the sea water beneath the Antarctica ice caps which had a unique habitat and salinity range due to the ice caps expelling large amount of salt through brine channels into the ocean below. The project consisted of two PCB boards, a probe to be inserted through a drilled hole in the ice and a controller to calibrate, request and display measurements. The probe used a complex switching array to measure the average resistance between a pair of either gold or titanium electrodes using a voltage sweep, then it would calculate the waters conductivity, and finally its salinity.

![The salinity sensor probe image](Report/Figures/probe_final.png)

##File Structure
- EEE4022S 2024 FINAL REPORT CLRCAM007 Clark CF - Pead: The final project report
- CAD: 3D printing files for the pressure sensor waterproof
- Code: The embedded C project code running on the controller and probe boards and the RS485 protocol between them.
- GA Forms: forms required to submit to ECSA
- PCBs: The KiCAD design files for the controller, probe and gold electrode PCBs.
- Presentation: Contains the power point and helper files for the final presentation
- Prototype: The prototype salinity sensor that preceeded this project.
- Report Helper Documents: Documents used for data processing and store from the testing of the probe.
- Report: The LaTeX project file for generating the final report.
