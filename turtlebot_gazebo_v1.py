import numpy as np
import matplotlib.pyplot as plt
import time
import csv
##################################################### Individual action ########################################################
def cost(Xn,Yn,Thetai,UL,UR):
    t = 0
    dt = 0.1
    Thetan = 3.14 * Thetai / 180
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        if (obstacle(Xs,Ys)):
            return 0,0,0,0
        Xn += 0.5*wheel_radius * (UL + UR) * np.cos(Thetan) * dt
        Yn += 0.5*wheel_radius * (UL + UR) * np.sin(Thetan) * dt
        Thetan += (wheel_radius / wheel_distance) * (UR - UL) * dt
        D += np.sqrt(np.power(0.5*wheel_radius*(UL+UR)*np.cos(Thetan)*dt, 2) + np.power(0.5*wheel_radius*(UL+UR)*np.sin(Thetan)*dt, 2))
        if(plot_nodes):
            plt.plot([Xs, Xn], [Ys, Yn], color="red")
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D
#################################################### obstactle space #########################################################
def obstacle(x,y):
    if(((x>=150-(clearance + bot_radius))  and (x<=165 + (clearance + bot_radius)) and (y<=200) and (y>=75 - (clearance + bot_radius)))   or    # for upper triangle
       ((x>=250-(clearance + bot_radius)) and (x<=265 + (clearance + bot_radius)) and (y>=0) and (y<=125 + (clearance + bot_radius))) or      # for lower rectangle
        (( 50 + (clearance + bot_radius) >= np.sqrt((x-400)**2 + (y-110)**2))) or                            # for circle
        (x<=(clearance + bot_radius)) or (x>=600-(clearance + bot_radius)) or (y<=(clearance + bot_radius)) or (y>=200-(clearance + bot_radius))):      # for boundary
        return 1
###################################################### create children #########################################################
def actions(OL_top_node, CL_nodes, final_x_config, final_y_config):
    children = []     #(tc,c2c,c2g, parent ID, x,y,theta,RPM1,RPM2)
    actions=[[0,RPM1], [RPM1,0], [RPM1,RPM1], [0,RPM2], [RPM2,0], [RPM2, RPM2], [RPM1,RPM2], [RPM2,RPM1]]
    for action in actions:
        new_x,new_y,theta,d= cost(OL_top_node[5],OL_top_node[6],OL_top_node[7], action[0],action[1])
        if(not (new_x==0 and new_y==0 and theta==0 and d==0)):  # checks if there is no obstacle
            val = np.where((np.round(CL_nodes[:, 5] / 4) * 4 == np.round(new_x / 4) * 4) & (np.round(CL_nodes[:, 6] / 4) * 4 == np.round(new_y / 4) * 4) & (np.round(CL_nodes[:, 7] / 180) * 180 == np.round(theta / 180) * 180))[0]  # check if node is in close list
            if(val.size==0):
                c2g = np.sqrt((final_x_config - new_x)**2 + (final_y_config - new_y)**2)
                c2c = OL_top_node[1]+d
                tc = c2g + c2c
                children.append([tc,c2c,c2g,OL_top_node[3] ,new_x, new_y, theta,action[0],action[1]]) # create child
    return children
#################################################### main ####################################
goal_threshold = 10
wheel_radius = 3.3      # in cm
bot_radius = 10.5       # in cm
wheel_distance = 16     # in cm
correct_input = False
while(not correct_input):           # take input from user until correct input is entered
    initial_x_config = float(input('enter starting x coordinate: '))    # 50
    initial_y_config = float(input('enter starting y coordinate: '))       # 100
    initial_theta_config = float(input('enter starting orientation: '))     # 0
    final_x_config = float(input('enter final x coordinate: '))         # 550
    final_y_config = float(input('enter final y coordinate: '))         # 100
    RPM1 = float(input('enter left wheel RPM: '))               # recommended: 6
    RPM2 = float(input('enter right wheel RPM: '))              # recommended: 3
    clearance = float(input('enter clearance: '))               # 5
    if(obstacle(initial_x_config,initial_y_config) or obstacle(final_x_config,final_y_config)):
        print('invalid input')
    else:
        correct_input=True
################################### initialization #######################
start_time = time.time()
plot_nodes=0             
OL_nodes = np.array([[0,0,0,1,0,initial_x_config, initial_y_config,initial_theta_config,0,0]])   # open list   (total_cost, c2c, c2g, node_id, parent_id, x, y, theta , RPM1, RPM2)
CL_nodes = np.array([[-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]])                                 # close list
NodeID = 1  
distance = np.sqrt((initial_x_config - final_x_config) ** 2 + (initial_y_config - final_y_config) ** 2)
print('finding shortest path.........')
################################## loop ###############################
while( (not (distance<=goal_threshold)) and (not OL_nodes.shape[0]==0)): # run loop until goal is reached or the open list becomes empty
    distance = np.sqrt((CL_nodes[-1][5] - final_x_config) ** 2 + (CL_nodes[-1][6] - final_y_config) ** 2)
    OL_nodes = OL_nodes[np.argsort(OL_nodes[:, 0])]
    children = actions(OL_nodes[0],CL_nodes, final_x_config, final_y_config)            # calls function to generate children 
    for i in range(len(children)): 
        val = np.where((np.round(OL_nodes[:, 5] / 4) * 4 == np.round(children[i][4] / 4) * 4) & (np.round(OL_nodes[:, 6] / 4) * 4 == np.round(children[i][5] / 4) * 4) & (np.round(OL_nodes[:, 7] / 180) * 180 == np.round(children[i][6] / 180) * 180))[0] # checks if child is present in open list
        if(val.size>0):
            if (children[i][1] < OL_nodes[int(val)][1]):  # checks if the child has a lower c2c      
                    OL_nodes[int(val)][0] = children[i][0]     # update total cost
                    OL_nodes[int(val)][1] = children[i][1]     # update cost to come
                    OL_nodes[int(val)][4] = children[i][3]     # update the parent
                    OL_nodes[int(val)][8] = children[i][7]     # update RPM1
                    OL_nodes[int(val)][9] = children[i][8]     # update RPM2
        else:
                OL_nodes = np.vstack([OL_nodes, [children[i][0],children[i][1],children[i][2],  NodeID+1,children[i][3],children[i][4],children[i][5],children[i][6],children[i][7],children[i][8]]])   # add the child to open list
                NodeID +=1
    CL_nodes = np.vstack([CL_nodes, OL_nodes[0]])   # pops the lowest cost to come element from open list and add it to closed list
    OL_nodes = np.delete(OL_nodes, 0, axis=0)
print('solution found')
end_time = time.time()
print(end_time-start_time)
############################################ Visualization######################################
plot_nodes=1
print('preparing graph.......')
fig, ax = plt.subplots()
plt.xlim(0,600)
plt.ylim(0,200)
############### obstacle space ########
x, y = np.meshgrid(np.arange(0, 600), np.arange(0, 200))
rect1 = (x>=150-(clearance + bot_radius)) & (x<=165 + (clearance + bot_radius)) & (y<=200) & (y>=75 - (clearance + bot_radius))
ax.fill(x[rect1], y[rect1], color='lightblue')
rect2 = (x>=250-(clearance + bot_radius)) & (x<=265 + (clearance + bot_radius)) & (y>=0) & (y<=125 + (clearance + bot_radius))
ax.fill(x[rect2], y[rect2], color='lightblue')
circle = ( 50 + (clearance + bot_radius) >= np.sqrt((x-400)**2 + (y-110)**2))
ax.fill(x[circle], y[circle], color='lightblue')
boundary1 = (x<=(clearance + bot_radius)) 
ax.fill(x[boundary1], y[boundary1], color='lightblue')
boundary2 = (x>=600-(clearance + bot_radius))
ax.fill(x[boundary2], y[boundary2], color='lightblue')
boundary3 = (y<=(clearance + bot_radius))
ax.fill(x[boundary3], y[boundary3], color='lightblue')
boundary4 = (y>=200-(clearance + bot_radius))
ax.fill(x[boundary4], y[boundary4], color='lightblue')
############ node exploration #########
for i in range(CL_nodes.shape[0]):
    val = np.where(CL_nodes[i][4] == CL_nodes[:, 3])[0]
    if(val.size>0):
        cost(CL_nodes[int(val)][5],CL_nodes[int(val)][6],CL_nodes[int(val)][7],CL_nodes[i][8],CL_nodes[i][9])
for i in range(OL_nodes.shape[0]):
    val = np.where(OL_nodes[i][4] == OL_nodes[:, 3])[0]
    if(val.size>0):
        cost(OL_nodes[int(val)][5],OL_nodes[int(val)][6],OL_nodes[int(val)][7],OL_nodes[i][8],OL_nodes[i][9])
######### backtrack #################
backtrack = np.array([[CL_nodes[-2][5],CL_nodes[-2][6],CL_nodes[-2][8],CL_nodes[-2][9]]])
node = CL_nodes[-2][4]
while (node):
    val = np.where(CL_nodes[:, 3] == node)[0]
    backtrack=np.vstack([backtrack, [CL_nodes[int(val)][5],CL_nodes[int(val)][6],CL_nodes[int(val)][8],CL_nodes[int(val)][9]]])
    ax.plot(CL_nodes[int(val)][5], CL_nodes[int(val)][6], color='blue', marker='o')
    node = CL_nodes[int(val)][4]
backtrack = np.flip(backtrack,axis = 0)
with open('rpm.csv', 'w', newline='') as file:      # writing in csv file
    writer = csv.writer(file)
    for i in range(backtrack.shape[0]):
        writer.writerow([backtrack[i][2], backtrack[i][3]])
####################################
plt.plot(initial_x_config, initial_y_config, color='blue', marker='o')
plt.plot(final_x_config, final_y_config, color='blue', marker='o')
plt.show()

