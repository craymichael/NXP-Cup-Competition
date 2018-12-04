# NXP Cup Competition Project
This repository encapsulates the code used in the Fall 2017 NXP cup at RIT. A line-following
algorithm is employed with PID control to navigate a NXP car through NXP tracks. Cornering
detection triggers braking and differential steering to drift through tight bends.

Fixing Errors Due to Device not Being Found:
```Error instantiating RTE components
Error #543: Device MK64FN1M0xxx12(NXP) not found, pack 'Keil.Kinetis_K60_DFP' is not installed
Error #540: 'Keil::Device:Startup:1.0.0' component is not available for target 'Target 1'```

Change the device from NXP K64F (1M) to Freescale K64F (1M) depending on Keil version
