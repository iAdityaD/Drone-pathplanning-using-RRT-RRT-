"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None

    #editted for 3d
    def __init__(self,
                 obstacle_list,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 use_funnel=True):
        """
        Setting Parameter

        start:Start Position [x,y,z]
        goal:Goal Position [x,y,z]
        obstacleList:obstacle Positions [[x,y,z,size],...]
        max_iter: maximum nuber of iteration in which to find the path

        """
        
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.goal_sample = 0
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.imposible = True
        self.use_funnel = use_funnel
       
    def prePlan(self, start, goal):
        """
        prePlan
        prepare for search. sets both the goal and start
        
        start: 3d-vector with start position
        goal: 3d-vetor with goal position
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1],  goal[2])
        self.goal_node = self.end
        #setup for conical search
        self.goalDist, goalTheta, goalPhi = self.calc_distance_and_angle(self.start,self.end)
        #print("ANGLESSSSSSSSSSS", self.goalDist, "theta ", goalTheta, "phi ", goalPhi)
        self.goalDist *= 1.1
        self.goalDir = np.array([(goal[0]-start[0])/self.goalDist, (goal[1]-start[1])/self.goalDist, (goal[2]-start[2])/self.goalDist])
        ct = math.cos(goalTheta)
        cp = math.cos(goalPhi)
        st = math.sin(goalTheta)
        sp = math.sin(goalPhi)
        R_z = np.array([[ct,-st,0], #WRONG ROTATION MATRIX, just for magical fixes
                        [st, ct,0],
                        [0,  0, 1]])
        R_y = np.array([[cp,0,sp],
                        [0, 1,0],
                        [-sp,0,cp]])
        self.R = np.matmul(R_y, R_z)

        self.constSinTheta = math.tan(goalTheta/2)
        
        for (ox, oy, oz, dx, dy, dz) in self.obstacle_list:
            ex = goal[0]-ox
            ey = goal[1]-oy
            ez = goal[2]-oz                  
            if ex >= 0 and ex <= dx and ey >= 0 and ey <= dy and ez >= 0 and ez <= dz:
                print("Goal is in a box")
                return None
        
        self.imposible = False
        
        
    #editted for 3d
    def planning(self, animation=False):
        """
        rrt path planning

        animation: flag for animation on or off
        """
        
        if(self.imposible == True):
            return None
        
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = None
            if self.use_funnel:
                rnd_node = self.get_random_funnel_node()
            else:
                rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y,
                                      self.node_list[-1].z) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    #editted for 3d
    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d, theta, phi = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta) * math.cos(phi)
            new_node.y += self.path_resolution * math.sin(theta) * math.cos(phi)
            new_node.z += self.path_resolution * math.sin(phi)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.path_z.append(to_node.z)
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.z = to_node.z

        new_node.parent = from_node
        #print(new_node.x, new_node.y, new_node.z)

        return new_node

    #editted for 3d
    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent
        path.append([node.x, node.y, node.z])

        return path

    #editted for 3d
    def calc_dist_to_goal(self, x, y, z):
        dx = x - self.end.x
        dy = y - self.end.y
        dz = z - self.end.z
        return math.hypot(math.hypot(dx,dy), dz)

    #Values are a bit larger than the size of the city in the simulation
    def get_random_node(self):
        return self.Node(random.uniform(-15, 15),
                         random.uniform(-15, 15),
                         random.uniform(-15, 15))
    
    #HOMEBREW
    def get_random_funnel_node(self, it):
        """
        get_random_node
        generate a random node from a conical field. 
        
        Returns random node or 'end' once every goal_sample_rate calls
        """
        
        if self.goal_sample == 0:
            self.goal_sample = self.goal_sample_rate
            return self.end
        else:
            self.goal_sample -= 1

        dist = self.goalDist
        if(it < 200): 
            dist = (self.goalDist*it)/200 + self.connect_circle_dist

        #TODO generate in a cone
        L = random.uniform(0, dist)
        if(L < self.connect_circle_dist):
            a = self.connect_circle_dist
            return self.Node(random.uniform(self.start.x-a, self.start.x+a),
                             random.uniform(self.start.y-a, self.start.y+a),
                             random.uniform(self.start.z-a, self.start.z+a))
        
        r = random.uniform(0, L*self.constSinTheta)
        p = random.uniform(0, 6.28)     #angle aroung goalDir
        #cone_center = self.goalDir*L
        rNode = np.matmul(self.R, np.array([L, r*math.cos(p), r*math.sin(p)]))

        #print(rNode)
        rnd = self.Node(rNode[0] + self.start.x, rNode[1] + self.start.y, rNode[2] + self.start.z)      
        
        #print(rNode)
        return rnd
    
    #ORIGINAL
    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    #ORIGINAL
    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    #editted for 3d
    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.z - rnd_node.z)**2
                 for node in node_list]
 
        return dlist.index(min(dlist))

    #edit for 3d and rectangular obstacles
    @staticmethod
    def check_collision(node, obstacleList):
        
        if node is None:
            print("Geen nits")
            return False
        
        if node.z <= 0:
            return False

        #calculate the distance between obstacle edges and node
        for (ox, oy, oz, dx, dy, dz) in obstacleList:
            dx_list = [x - ox for x in node.path_x]
            dy_list = [y - oy for y in node.path_y]
            dz_list = [z - oz for z in node.path_z]
            
            for ex,ey,ez in zip(dx_list, dy_list, dz_list):
                if ex >= 0 and ex <= dx and ey >= 0 and ey <= dy and ez >= 0 and ez <= dz:
                    return False
                
        return True  # safe

    #editted for 3d
    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        theta = math.atan2(dy, dx)
        phi = math.atan2(dz, math.hypot(dx, dy))
        return d, theta, phi
