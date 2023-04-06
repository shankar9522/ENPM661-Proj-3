import numpy as np
import matplotlib.pyplot as plt
import math
#################################################### obstacle map visualization #######################################
fig, ax = plt.subplots()
plt.xlim(0,600)
plt.ylim(0,250)
x, y = np.meshgrid(np.arange(0, 600), np.arange(0, 250))
rect1 = (x>=95) & (x<=155) & (y>=0) & (y<=105)
ax.fill(x[rect1], y[rect1], color='lightblue')
rect2 = (x>=95) & (x<=155) & (y>=145) & (y<=250)
ax.fill(x[rect2], y[rect2], color='lightblue')
hex = (x >= (235 - 5)) & (x <= (365 + 5)) & ((x + 2*y) >= 395) & ((x - 2*y) <= 205) & ((x - 2*y) >= -105) & ((x + 2*y) <= 705)
ax.fill(x[hex], y[hex], color='lightblue')
tri = (y >= 1.75*x - 776.25) & (y <= -1.75*x + 1026.25) & (x >= 455)
ax.fill(x[tri], y[tri], color='lightblue')
boundary1 = (x<=5) 
ax.fill(x[boundary1], y[boundary1], color='lightblue')
boundary2 = (x>=595)
ax.fill(x[boundary2], y[boundary2], color='lightblue')
boundary3 = (y<=5)
ax.fill(x[boundary3], y[boundary3], color='lightblue')
boundary4 = (y>=245)
ax.fill(x[boundary4], y[boundary4], color='lightblue')
##################################################### Individual action ########################################################
def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    dt = 0.015
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    x = []
    y = []
    D=0
    while t<0.1:
        t = t + dt
        Xn += 0.5*wheel_radius * (UL + UR) * np.cos(Thetan) * dt
        Yn += 0.5*wheel_radius * (UL + UR) * np.sin(Thetan) * dt
        Thetan += (wheel_radius / wheel_distance) * (UR - UL) * dt
        D+= np.sqrt(math.pow((0.5*wheel_radius * (UL + UR) * np.cos(Thetan) * dt),2)+math.pow((0.5*wheel_radius * (UL + UR) * np.sin(Thetan) * dt),2))
        x.append(Xn)
        y.append(Yn)
    Thetan = 180 * (Thetan) / 3.14
    return x, y, Thetan, D
#################################################### create obstactle space #########################################################
def obstacle(x,y):
    if((x>=95 and x<=155 and y>=0 and y<=105)   or (x>=95 and x<=155 and y>=145 and y<=250) or      # for upper and lower rectangle
        (y >= 1.75*x - 776.25 and y <= -1.75*x + 1026.25 and x >= 455) or                            # for trxangle
        ((x >= (235 - 5)) and (x <= (365 + 5)) and ((x + 2*y) >= 395) and ((x - 2*y) <= 205) and ((x - 2*y) >= -105) and ((x + 2*y) <= 705))  or  # for hexagon
        (x<=5) or (x>=595) or (y<=5) or (y>=245) ):      # for boundary
        return 1
###################################################### create children #################################################################
def actions(OL_top_node, CL_nodes, step_size, final_x_config, final_y_config):
    children = []     #(tc,c2c,c2g, parent ID, x,y,theta,x_plot,y_plot)
    actions=[[0,RPM1], [RPM1,0], [RPM1,RPM1], [0,RPM2], [RPM2,0], [RPM2, RPM2], [RPM1,RPM2], [RPM2,RPM1]]
             
    for action in actions:
        x,y,theta,d= cost(OL_top_node[5],OL_top_node[6],OL_top_node[7], action[0],action[1])

        if(not obstacle(x[-1], y[-1])):  # checks if there is no obstacle
            val = np.where((np.round(CL_nodes[:, 5] * 2) / 2 == np.round(x[-1] * 2) / 2) & (np.round(CL_nodes[:, 6] * 2) / 2 == np.round(y[-1] * 2) / 2) & (np.round(CL_nodes[:, 7] / 30) * 30 == np.round(theta / 30) * 30))[0]  # check if node is in close list
            if(val.size==0):
                c2g = np.sqrt((final_x_config - x[-1])**2 + (final_y_config - y[-1])**2)
                c2c = OL_top_node[1]+d
                tc = c2g + c2c
                children.append([tc,c2c,c2g,OL_top_node[3] ,x[-1], y[-1],theta]) # create child
                # for i in range(1,len(x)):
                #     plt.plot([x[i-1],x[i]],[y[i-1],y[i]],color="blue")
                # # plt.pause(0.1)
    return children
#################################################### main ####################################
step_size = 5
final_x_config = 50
final_y_config = 200
initial_x_config = 15
initial_y_config = 15
initial_theta_config = 0
RPM1 = 50
RPM2 = 100
wheel_radius = 0.038 *10
bot_radius = 70 
wheel_distance = 0.354 *10
# ################################################################## Start loop#######################
OL_nodes = np.array([[0,0,0,1,0,initial_x_config, initial_y_config,initial_theta_config]])        # open list   (total_cost, c2c, c2g, node_id, parent_id, x, y, theta)
CL_nodes = np.array([[-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]])                                 # close list
NodeID = 1  
radius = 1.5*step_size      # modify this
distance = np.sqrt((initial_x_config - final_x_config) ** 2 + (initial_y_config - final_y_config) ** 2)
while( (not distance<=radius) and  (not OL_nodes.shape[0]==0)): # run loop until goal is reached or the open list becomes empty
    distance = np.sqrt((CL_nodes[-1][5] - final_x_config) ** 2 + (CL_nodes[-1][6] - final_y_config) ** 2)
    OL_nodes = OL_nodes[OL_nodes[:,0].argsort()]        # arrange open list according to total cost
    children = actions(OL_nodes[0],CL_nodes, step_size, final_x_config, final_y_config)            # calls function to generate children 

    for i in range(len(children)): 
        val = np.where((np.round(OL_nodes[:, 5] * 2) / 2 == np.round(children[i][4] * 2) / 2) & (np.round(OL_nodes[:, 6] * 2) / 2 == np.round(children[i][5] * 2) / 2) & (np.round(OL_nodes[:, 7] / 30) * 30 == np.round(children[i][6] / 30) * 30))[0] # checks if child is present in open list
        if(val.size>0):
            if (children[i][0] < OL_nodes[int(val)][0]):  # checks if the child has a lower cost to come than earlier      
                    OL_nodes[int(val)][0] = children[i][0]     # update total cost
                    OL_nodes[int(val)][0] = children[i][2]     # update cost to go
                    OL_nodes[int(val)][0] = children[i][1]     # update cost to come
                    OL_nodes[int(val)][2] = children[i][3]     # update the parent
        else:
                OL_nodes = np.vstack([OL_nodes, [children[i][0],children[i][1],children[i][2],  NodeID+1,children[i][3],children[i][4],children[i][5],children[i][6]]])   # add the child to open list
                NodeID +=1
    CL_nodes = np.vstack([CL_nodes, OL_nodes[0]])   # pops the lowest cost to come element from open list and add it to closed list
    OL_nodes = np.delete(OL_nodes, 0, axis=0)
    # plt.pause(0.1)
print('solution found')
ax.scatter(initial_x_config, initial_y_config, color='red')
ax.scatter(final_x_config, final_y_config, color='red')
plt.show()


