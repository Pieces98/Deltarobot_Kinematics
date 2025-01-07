import numpy as np
from numpy import cos, sin, pi, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

#####---------------------Value Setting---------------------#####
eps = 1e-9              #If the absolute value is less than eps, the value is treated as 0
t = np.linspace(0, 2*pi, 200)
XYZcoor = np.array([500*cos(t), 500*sin(t), -1000+200*sin(4*t)], dtype='float')

Aleg = np.array([524, 524, 524], dtype='float')         #Set active link length
Pleg = np.array([1244, 1244, 1244], dtype='float')      #Set passive link length

sideB = 567                         #Set base length
sideP = 76                          #Set platform length
show_result = True                  #True : Showing individual results
save_in_gif = True                  #True : save plot to gif file

#####---------------------Calculation Section(IPK)---------------------#####
print("Start Calculation(IPK)")

num_set = XYZcoor.shape[1]
res_angle = np.zeros((3, num_set))

a = sqrt(3)/6*sideB-sqrt(3)/3*sideP
b = sideP/2-sideB/4
c = sqrt(3)/6*sideP-sqrt(3)/12*sideB

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

P1 = 2*Aleg[0]*(XYZcoor[1,:]+a)
Q1 = 2*Aleg[0]*XYZcoor[2,:]
R1 = (XYZcoor**2).sum(axis=0)+a**2+Aleg[0]**2-Pleg[0]**2+2*a*XYZcoor[1,:]
deg1 = PQR_Solver(P1, Q1, R1)

#print(np.around(np.rad2deg(deg1), 2))

P2 = -Aleg[1]*(sqrt(3)*(XYZcoor[0,:]+b)+XYZcoor[1,:]+c)
Q2 = 2*Aleg[1]*XYZcoor[2,:]
R2 = (XYZcoor**2).sum(axis=0)+b**2+c**2+Aleg[1]**2-Pleg[1]**2+2*(XYZcoor[0,:]*b+XYZcoor[1,:]*c)
deg2 = PQR_Solver(P2, Q2, R2)

#print(np.around(np.rad2deg(deg2), 2))

P3 = Aleg[2]*(sqrt(3)*(XYZcoor[0,:]-b)-XYZcoor[1,:]-c)
Q3 = 2*Aleg[2]*XYZcoor[2,:]
R3 = (XYZcoor**2).sum(axis=0)+b**2+c**2+Aleg[1]**2-Pleg[1]**2+2*(-XYZcoor[0,:]*b+XYZcoor[1,:]*c)
deg3 = PQR_Solver(P3, Q3, R3)

#print(np.around(np.rad2deg(deg3), 2))

deg_Aleg = np.array([deg1, deg2, deg3])

if show_result:
    for idx in range(XYZcoor.shape[1]):
        str1 = "{:0d}th Input Coordinate : <{:.4f}    {:.4f}    {:.4f}>".format(idx+1, XYZcoor[0,idx], XYZcoor[1,idx], XYZcoor[2,idx])
        print(str1)
        str2 = "Result Angle : <{:.2f}    {:.2f}    {:.2f}>\n".format(np.rad2deg(deg_Aleg[0,idx]), np.rad2deg(deg_Aleg[1,idx]), np.rad2deg(deg_Aleg[2,idx]))
        print(str2)


#####---------------------Calculation Section(FPK)---------------------#####
print("Start Calculation(FPK)")

num_set = deg_Aleg.shape[1]         #Number of angle set
Ai_coor = np.zeros((3,3,num_set))
Bi_coor = np.zeros((3,3,num_set))
Pi_coor = np.zeros((3,3,num_set))
res_coor = np.zeros((3,num_set))    #Variable to save results

#B-based coordinate Bi
vecs_Bi = np.array([[0, -sqrt(3)/6*sideB, 0],
                    [sideB/4, sqrt(3)/12*sideB, 0],
                    [-sideB/4, sqrt(3)/12*sideB, 0]])

Bi_coor = np.repeat(vecs_Bi[:,:,np.newaxis], num_set, axis=2)
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
    Ai_coor[:,:,i] = vecs_Ai
    #Coordinate shifting to calculate center of 3 spheres
    vecs_Aj = vecs_Ai-vecs_Pi_Pxyz

    
    if (np.abs(np.linalg.det(vecs_Aj)) >= 1):     #If 3 vectors are independents to each other (Matrix Rank = 3)
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
    Pi_coor[:,:,i] = np.array([sol, sol, sol])+vecs_Pi_Pxyz

print("Calculation Complete")

#####---------------------Plot Section---------------------#####
print("Start Plotting")
fig1 = plt.figure()
ax1 = Axes3D(fig=fig1, azim=-60, elev=30)
#azim : 0(parallel to X axis), 90(parallel to Y axis)
#elev : 0(parallel to XY plane), 90(perpendicular to XY plane)
Base_shape = np.array([[sideB/2, -sqrt(3)/6*sideB, 0],
                           [0, sqrt(3)/3*sideB, 0],
                           [-sideB/2, -sqrt(3)/6*sideB, 0]])
Base_verts = np.append(Base_shape, np.array([Base_shape[0,:]]), axis=0)

def update_linkage(t, data_set, plt_info):
    idx = t%num_set
    for list_idx, plot_data in enumerate(zip(plt_info, data_set)):
        plot, data = plot_data
        if(list_idx<=2):
            plot.set_data(data[:,0,idx], data[:,1,idx])
            plot.set_3d_properties(data[:,2,idx])
        else:
            plot.set_data(data[:,0,idx], data[:,1,idx])
            plot.set_3d_properties(data[:,2,idx])
    return plt_info

legs_data = [np.array([Bi_coor[i,:,:], Ai_coor[i,:,:], Pi_coor[i,:,:]]) for i in range(3)]
platform_data = [np.append(Pi_coor, np.array([Pi_coor[0,:,:]]), axis=0)]
data_set = legs_data.copy()
data_set.extend(platform_data)

legs = [ax1.plot(coor[0,0,0:1], coor[0,1,0:1], coor[0,2,0:1], '-ok', lw=1, ms=1)[0] for coor in legs_data]
platform_verts = [ax1.plot(coor[0,0,0:1], coor[0,0,0:1], coor[0,0,0:1], '-ok', lw=1, ms=1)[0] for coor in platform_data]
plt_info  = legs.copy()
plt_info.extend(platform_verts)

ax1.set_xlim3d([-800, 800])
ax1.set_xlabel('X')
ax1.set_ylim3d([-800, 800])
ax1.set_ylabel('Y')
ax1.set_zlim3d([-1500, 0])
ax1.set_zlabel('Z')

line_animation = animation.FuncAnimation(fig1, update_linkage, num_set, fargs=(data_set, plt_info), interval=1000/num_set, blit=True)
#blit True, False both available

ax1.plot(Base_verts[:,0], Base_verts[:,1], Base_verts[:,2], '-k')
ax1.plot(res_coor[0,:], res_coor[1,:], res_coor[2,:], '-r', lw=0.5)
if save_in_gif:
    print("Converting to GIF file...")
    line_animation.save('exAnimation.gif', writer='imagemagick', fps=30, dpi=100)
    print("Done")
print("Work Done")
plt.show(block=False)

fig2 = plt.figure()
ax_coor = fig2.add_subplot(2, 1, 1)
ax_angle= fig2.add_subplot(2, 1, 2)

steps = np.arange(0, num_set)

ax_coor.plot(steps, XYZcoor[0,:], '-r', lw=1, label='X')
ax_coor.plot(steps, XYZcoor[1,:], '-g', lw=1, label='Y')
ax_coor.plot(steps, XYZcoor[2,:], '-b', lw=1, label='Z')
ax_coor.grid(b=True, which='major', color='k', linestyle='-', alpha=0.2)
ax_coor.grid(b=True, which='minor', color='k', linestyle='--', alpha=0.1)
ax_coor.set_xlabel('Step')
ax_coor.set_ylabel('XYZ coordinate (mm)')
ax_coor.legend()


ax_angle.plot(steps, np.rad2deg(deg1), '-r', lw=1, label='deg1')
ax_angle.plot(steps, np.rad2deg(deg2), '-g', lw=1, label='deg2')
ax_angle.plot(steps, np.rad2deg(deg3), '-b', lw=1, label='deg3')
ax_angle.grid(b=True, which='major', color='k', linestyle='-', alpha=0.2)
ax_angle.grid(b=True, which='minor', color='k', linestyle='--', alpha=0.1)
ax_angle.set_xlabel('Step')
ax_angle.set_ylabel('Joint Angle (degree)')
ax_angle.legend()

plt.show(block=True)

















