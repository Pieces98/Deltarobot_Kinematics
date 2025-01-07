import numpy as np
from numpy import cos, sin, pi, sqrt

#####---------------------Value Setting---------------------#####
eps = 1e-9              #If the absolute value is less than eps, the value is treated as 0

#Set input <x, y, z> coordinate
XYZcoor = np.array([[0, 108.14, 0, 300.46, 0],
                    [0, -180.35, 0, 499.94, 0],
                    [-1064.45, -1243.54, -900.32, -1099.83, -1762.22]], dtype='float')

Aleg = np.array([524, 524, 524], dtype='float')         #Set active link length
Pleg = np.array([1244, 1244, 1244], dtype='float')      #Set passive link length

sideB = 567                         #Set base length
sideP = 76                          #Set platform length

#True : Showing individual results, False : Not showing individual results
show_result = True                  

#####---------------------Calculation Section---------------------#####

num_set = XYZcoor.shape[1]                  #Number of angle set
res_angle = np.zeros((3, num_set))          #Variable to save results
    
a = sqrt(3)/6*sideB-sqrt(3)/3*sideP         #Constant 1
b = sideP/2-sideB/4                         #Constant 2
c = sqrt(3)/6*sideP-sqrt(3)/12*sideB        #Constant 3

def PQR_Solver(Parr, Qarr, Rarr, sigma=1):
    root_term = Parr**2+Qarr**2-Rarr**2
    if np.any(root_term<0):
        print("Complex Term Exist")
        return np.full_like(Parr, np.nan)
    else:
        C_deg = (-Parr*Rarr-sigma*Qarr*sqrt(Parr**2+Qarr**2-Rarr**2))/(Parr**2+Qarr**2)
        S_deg = (-Qarr*Rarr+sigma*Parr*sqrt(Parr**2+Qarr**2-Rarr**2))/(Parr**2+Qarr**2)
        deg = np.arctan2(S_deg, C_deg)
        return deg

#degree 1
P1 = 2*Aleg[0]*(XYZcoor[1,:]+a)
Q1 = 2*Aleg[0]*XYZcoor[2,:]
R1 = (XYZcoor**2).sum(axis=0)+a**2+Aleg[0]**2-Pleg[0]**2+2*a*XYZcoor[1,:]
deg1 = PQR_Solver(P1, Q1, R1)

#degree 2
P2 = -Aleg[1]*(sqrt(3)*(XYZcoor[0,:]+b)+XYZcoor[1,:]+c)
Q2 = 2*Aleg[1]*XYZcoor[2,:]
R2 = (XYZcoor**2).sum(axis=0)+b**2+c**2+Aleg[1]**2-Pleg[1]**2+2*(XYZcoor[0,:]*b+XYZcoor[1,:]*c)
deg2 = PQR_Solver(P2, Q2, R2)

#degree 3
P3 = Aleg[2]*(sqrt(3)*(XYZcoor[0,:]-b)-XYZcoor[1,:]-c)
Q3 = 2*Aleg[2]*XYZcoor[2,:]
R3 = (XYZcoor**2).sum(axis=0)+b**2+c**2+Aleg[1]**2-Pleg[1]**2+2*(-XYZcoor[0,:]*b+XYZcoor[1,:]*c)
deg3 = PQR_Solver(P3, Q3, R3)

#Save
deg_set = np.array([deg1, deg2, deg3])
deg_set[np.abs(deg_set)<eps] = 0

if show_result:
    for idx in range(XYZcoor.shape[1]):
        str1 = "{:0d}th Input Coordinate : <{:.4f}    {:.4f}    {:.4f}>".format(idx+1, XYZcoor[0,idx], XYZcoor[1,idx], XYZcoor[2,idx])
        print(str1)
        str2 = "Result Angle : <{:.2f}    {:.2f}    {:.2f}>\n".format(np.rad2deg(deg_set[0,idx]), np.rad2deg(deg_set[1,idx]), np.rad2deg(deg_set[2,idx]))
        print(str2)

print("Calculation Complete")
















