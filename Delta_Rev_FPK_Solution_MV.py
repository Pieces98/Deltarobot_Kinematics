import numpy as np
from numpy import cos, sin, pi, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#####---------------------Value Setting---------------------#####
eps = 1e-9              #If the absolute value is less than eps, the value is treated as 0

#Set input angle in degree
deg_Aleg = np.deg2rad(np.array([[0, 10, -20.5, 47.5],
                                [0, 20, -20.5, -11.6],
                                [0, 30, -20.5, 21.4]]))

Aleg = np.array([524, 524, 524], dtype='float')         #Set active link length
Pleg = np.array([1244, 1244, 1244], dtype='float')      #Set passive link length

sideB = 567                         #Set base length
sideP = 76                          #Set platform length
show_result = True                  #True : Showing individual results, False : Not showing individual results

#####---------------------Calculation Section---------------------#####
num_set = deg_Aleg.shape[1]         #Number of angle set
res_coor = np.zeros((3,num_set))    #Variable to save results

#B-based coordinate Bi
vecs_Bi = np.array([[0, -sqrt(3)/6*sideB, 0],
                    [sideB/4, sqrt(3)/12*sideB, 0],
                    [-sideB/4, sqrt(3)/12*sideB, 0]])

#P-based coordinate Pi
vecs_Pi_Pxyz = np.array([[0, -sqrt(3)/3*sideP, 0],
                         [sideP/2, sqrt(3)/6*sideP, 0],
                         [-sideP/2, sqrt(3)/6*sideP, 0]])

#Loop for calculate individual coordinates
for i in range(num_set):
    #B-based vectors Li
    vecs_Li = np.array([[0, -Aleg[0]*cos(deg_Aleg[0,i]), -Aleg[0]*sin(deg_Aleg[0,i])],
                       [Aleg[1]*cos(deg_Aleg[1,i])*cos(pi/6), Aleg[1]*cos(deg_Aleg[1,i])*sin(pi/6), -Aleg[1]*sin(deg_Aleg[1,i])],
                       [-Aleg[2]*cos(deg_Aleg[2,i])*cos(pi/6), Aleg[2]*cos(deg_Aleg[2,i])*sin(pi/6), -Aleg[2]*sin(deg_Aleg[2,i])]])

    #B-based vectors Ai
    vecs_Ai = vecs_Bi+vecs_Li
    #Coordinate shifting to calculate center of 3 spheres
    vecs_Aj = vecs_Ai-vecs_Pi_Pxyz

    
    if (np.abs(np.linalg.det(vecs_Aj)) >= 0.001):     #If 3 vectors are independents to each other (Matrix Rank = 3)
        vec_e = np.array([1, 1, 1]).T
        vec_b = np.array([np.dot(vecs_Aj[0,:], vecs_Aj[0,:])-Pleg[0]**2, np.dot(vecs_Aj[1,:], vecs_Aj[1,:])-Pleg[1]**2, np.dot(vecs_Aj[2,:], vecs_Aj[2,:])-Pleg[2]**2]).T
        vec_u = np.matmul(np.linalg.inv(vecs_Aj), vec_e)
        vec_v = np.matmul(np.linalg.inv(vecs_Aj), vec_b)

        coef = [np.dot(vec_u, vec_u), 2*np.dot(vec_u, vec_v)-4, np.dot(vec_v, vec_v)]   #Coefficients of quadratic equation
        r_set = np.roots(coef)              #Solve quadratic equation

        sol1 = 0.5*(r_set[0]*vec_u+vec_v)   #Solution Coordinate 1
        sol2 = 0.5*(r_set[1]*vec_u+vec_v)   #Solution Coordinate 2
        sol = np.zeros((3,))
        
        #Select a result with smaller Z values
        if sol1[2] < sol2[2]:
            sol = sol1
        else:
            sol = sol2
        
    else:                                           #If 3 vectors are dependents to each other (Matrix Rank < 3)
        add_factor = (np.max(vecs_Aj) + np.min(vecs_Aj))/2      #Select add_factor to coordinate shifting
        
        #Coordinate shifting to make independent 3 vectors (make matrix rank to 3)
        vecs_Ak = vecs_Aj+np.array([[0, 0, add_factor],
                                    [0, 0, add_factor],
                                    [0, 0, add_factor]])
        
        vec_e = np.array([1, 1, 1]).T
        vec_b = np.array([np.dot(vecs_Ak[0,:], vecs_Ak[0,:])-Pleg[0]**2, np.dot(vecs_Ak[1,:], vecs_Ak[1,:])-Pleg[1]**2, np.dot(vecs_Ak[2,:], vecs_Ak[2,:])-Pleg[2]**2]).T
        vec_u = np.matmul(np.linalg.inv(vecs_Ak), vec_e)
        vec_v = np.matmul(np.linalg.inv(vecs_Ak), vec_b)
        
        coef = [np.dot(vec_u, vec_u), 2*np.dot(vec_u, vec_v)-4, np.dot(vec_v, vec_v)]   #Coefficients of quadratic equation
        r_set = np.roots(coef)              #Solve quadratic equation

        sol1 = 0.5*(r_set[0]*vec_u+vec_v)-np.array([0, 0, add_factor])      #Solution Coordinate 1
        sol2 = 0.5*(r_set[1]*vec_u+vec_v)-np.array([0, 0, add_factor])      #Solution Coordinate 2
        sol = np.zeros((3,))

        #Select a result with smaller Z values
        if sol1[2] < sol2[2]:
            sol = sol1
        else:
            sol = sol2
    
    sol[np.abs(sol)<eps] = 0        #Treats coordinate less than eps as 0
    res_coor[:,i] = sol             #Save result

    #Showing results
    if show_result:
        str1 = "{:0d}th Input Degree : {:.8f}    {:.8f}    {:.8f}".format(i+1, np.rad2deg(deg_Aleg[0,i]), np.rad2deg(deg_Aleg[1,i]), np.rad2deg(deg_Aleg[2,i]))
        print(str1)
        str2 = "Result Coordinate : <{:.4f}    {:.4f}    {:.4f}>\n".format(sol[0], sol[1], sol[2])
        print(str2)

print("Calculation Complete")

#####---------------------Plot Section---------------------#####
fig = plt.figure()
ax = fig.gca(projection='3d')
#ax.pbaspect = [1.0, 1.0, 1.0]
Base_shape = np.array([[sideB/2, -sqrt(3)/6*sideB, 0],
                           [0, sqrt(3)/3*sideB, 0],
                           [-sideB/2, -sqrt(3)/6*sideB, 0]])
Base_verts = np.append(Base_shape, np.array([Base_shape[0,:]]), axis=0)

ax.plot(Base_verts[:,0], Base_verts[:,1], Base_verts[:,2], '-k')
ax.plot(res_coor[0,:], res_coor[1,:], res_coor[2,:], 'x')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

