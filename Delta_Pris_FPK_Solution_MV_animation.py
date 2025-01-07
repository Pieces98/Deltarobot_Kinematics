import numpy as np
from numpy import cos, sin, pi, sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

def Triangle_shape(len_side, zval = (0, 0, 0)):
    return np.array([[-len_side/2, -sqrt(3)/6*len_side, zval[0]], [len_side/2, -sqrt(3)/6*len_side, zval[1]], [0, sqrt(3)/3*len_side, zval[2]]])

#####---------------------Value Setting---------------------#####
eps = 1e-9              #If the absolute value is less than eps, the value is treated as 0

Aleg_limit = np.array([[67, 479],
                       [67, 479],
                       [67, 479]], dtype='float')

#Set input length of each prismatic leg
t = np.linspace(0, 2*pi, 200)
Aleg = np.array([250+150*sin(t), 250+150*cos(2*t), 250+150*cos(3*t)], dtype='float')

Pleg = np.array([264, 264, 264], dtype='float')      #Set passive link length

sideB = 350                         #Set base length
sideP = 30                         #Set platform length
sideC = 432                         #Set case length
poleC = 1000                        #Set case height
show_result = False                 #True : Showing individual results, False : Not showing individual results
save_in_gif = True                  #True : save plot to gif file
#####---------------------Calculation Section---------------------#####
print("Start Calculation")

num_set = Aleg.shape[1]
Ai_coor = np.zeros((3,3,num_set))
Pi_coor = np.zeros((3,3,num_set))
res_coor = np.zeros((3,num_set))    #Variable to save results

vecs_Pi_Pxyz = Triangle_shape(sideP)

for i in range(num_set):
    vecs_Ai = Triangle_shape(sideB, -Aleg[:,i])
    Ai_coor[:,:,i] = vecs_Ai
    
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

    if show_result:
        str1 = "{:0d}th Input Length : {:.4f}    {:.4f}    {:.4f}".format(i+1, Aleg[0,i], Aleg[1,i], Aleg[2,i])
        print(str1)
        str2 = "Result Coordinate : <{:.4f}    {:.4f}    {:.4f}>\n".format(sol[0], sol[1], sol[2])
        print(str2)

print("Calculation Complete")
#####---------------------Plot Section---------------------#####
print("Start Plotting")
fig = plt.figure()
ax = Axes3D(fig=fig, azim=15, elev=25)

Case_cover_shape = Triangle_shape(sideC)
Case_base_shape = Triangle_shape(sideC, (-poleC, -poleC, -poleC))
Pole_shape = np.array([Triangle_shape(sideB), Triangle_shape(sideB, -Aleg_limit[:,0]), Triangle_shape(sideB, -Aleg_limit[:,1]),Triangle_shape(sideB, (-poleC, -poleC, -poleC))])

Case_cover_verts = np.append(Case_cover_shape, np.array([Case_cover_shape[0,:]]), axis=0)
Case_base_verts = np.append(Case_base_shape, np.array([Case_base_shape[0,:]]), axis=0)
ax.plot(Case_cover_verts[:,0], Case_cover_verts[:,1], Case_cover_verts[:,2], '-k', lw=1.5)
ax.plot(Case_base_verts[:,0], Case_base_verts[:,1], Case_base_verts[:,2], '-k', lw=1.5)
for idx in range(3):
    ax.plot(Pole_shape[:,idx,0], Pole_shape[:,idx,1], Pole_shape[:,idx,2], '--ok', lw=1.5, ms=5)

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

legs_data = [np.array([Ai_coor[i,:,:], Pi_coor[i,:,:]]) for i in range(3)]
platform_data = [np.append(Pi_coor, np.array([Pi_coor[0,:,:]]), axis=0)]
data_set = legs_data.copy()
data_set.extend(platform_data)

legs = [ax.plot(coor[0,0,0:1], coor[0,1,0:1], coor[0,2,0:1], '-ok', lw=1, ms=1)[0] for coor in legs_data]
platform_verts = [ax.plot(coor[0,0,0:1], coor[0,0,0:1], coor[0,0,0:1], '-ok', lw=1, ms=1)[0] for coor in platform_data]
plt_info  = legs.copy()
plt_info.extend(platform_verts)

ax.set_xlim3d([-300, 300])
ax.set_xlabel('X')
ax.set_ylim3d([-300, 300])
ax.set_ylabel('Y')
ax.set_zlim3d([-poleC, 0])
ax.set_zlabel('Z')

line_animation = animation.FuncAnimation(fig, update_linkage, num_set, fargs=(data_set, plt_info), interval=10000/num_set, blit=True)
#blit True, False both available

ax.plot(res_coor[0,:], res_coor[1,:], res_coor[2,:], '-r', lw=0.5)
if save_in_gif:
    print("Converting to GIF file...")
    line_animation.save('exAnimation.gif', writer='imagemagick', fps=30, dpi=100)
    print("Done")
print("Work Done")
plt.show()


















