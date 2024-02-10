# Improved Continuous Fast Terminal Sliding Mode Control For Speed Regulation Of PMSM Drive System
This Project focuses on developing and simulating a PMSM drives system with Continuous Fast Terminal  Sliding Mode Control (CFTSMC) with Simulink and CMEX program. The system used are based on the research articles by W. Xu er al. and L. Qu et al. The commonly used SMC controllers are conventional SMC and integral SMC, both of which are used for speed or current control. To address the drawbacks of SMC methods, such as high switching gain and chattering levels, the CFTSMC method has advantages in determining errors faster, providing higher tracking accuracy, and being resistant to external load disturbances.

### PMSM Model
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/7b426c2b-1d00-474e-85a0-82000e52aa4c)

## SMC
### Equation
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/76b1a91a-65b0-4e74-97c4-635f239e1971)

![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/20a09371-bed8-4030-bb3b-e757f4fd8d1b)

### Simulink Block
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/d3d3c60a-79e3-49a4-aedd-9a806bf1f744)
### Parameter 
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/118ba38a-b11c-4fe0-8bb2-0491f4fe0f1b)


### Simulation Results
Load Torque: 5 Nm at t = 5 Reference Speed: 0 to 20 rpm on t = 1 second and 20 to 10 rpm on t = 3 second
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/2d21f600-085f-4eb6-974d-d1568d1c10bb)


## CFTSMC
### Equation
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/45e53197-3b54-459c-8f3e-7aea9bd84b68)

### Simulink Block
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/95b83c40-954d-45e4-b7c2-a7aaf6c51d06)
### Parameter 
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/1dda417a-70e7-433f-ab28-b902233549e2)

### Simulation Results
Load Torque: 5 Nm at t = 5 Reference Speed: 0 to 20 rpm on t = 1 second and 20 to 10 rpm on t = 3 second
![image](https://github.com/adibharyo/improved-cftsmc-pmsm/assets/70692957/e114c486-09d0-469b-ad88-dbbc99b322cc)

