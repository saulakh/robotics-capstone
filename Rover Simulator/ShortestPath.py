#!/usr/bin/python

import numpy as np
import yaml
import matplotlib.pyplot as plt

def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    [nrows,ncols] = np.shape(occupancy_map)

    x0 = start[0]
    y0 = start[1]
    xf = goal[0]
    yf = goal[1]

    j0 = int(np.rint(x0/x_spacing - 0.5))
    i0 = int(np.rint(y0/y_spacing - 0.5))
    jend = int(np.rint(xf/x_spacing - 0.5))
    iend = int(np.rint(yf/y_spacing - 0.5))

    graphmap = np.zeros((nrows,ncols))
    graphmap[occupancy_map==0] = 1
    graphmap[occupancy_map==1] = 2
    graphmap[i0,j0] = 5
    graphmap[iend,jend] = 6

    parent = np.zeros((nrows,ncols))
    distanceFromStart = np.matrix(np.ones((nrows,ncols)) * np.inf)

    distanceFromStart[i0,j0] = 0
    numExpanded = 0

    loc = np.array([[i0,iend],[j0,jend]])
    st_node = np.ravel_multi_index(loc,(nrows,ncols))
    start_node = int(st_node[0])
    dest_node = int(st_node[1])
    
    while True:
      
        min_dist = np.amin(distanceFromStart.flatten())
        current = np.argmin(distanceFromStart.flatten())

        if (current==st_node[-1]) or np.isinf(min_dist):
                    break
                
        current_ind = np.unravel_index(current,(nrows,ncols))
        graphmap[current_ind] = 3
        distanceFromStart[current_ind] = np.inf
        numExpanded +=1
        """
        plt.imshow(graphmap)
        plt.show()
        """   
        i = current_ind[0]; j = current_ind[1]
        for jw in range(j-1,j+2,2):
                    if i>=0 and i<=nrows-1 and jw>=0 and jw<=ncols-1:
                        if (graphmap[i,jw]!=2) and (graphmap[i,jw]!=3) and (graphmap[i,jw]!=5):
                            graphmap[i,jw] = 4
                            if distanceFromStart[i,jw] >= min_dist+1:
                                distanceFromStart[i,jw] = min_dist+1
                            parent[i,jw] = current
        for iw in range(i-1,i+2,2):
                    if iw>=0 and iw<=nrows-1 and j>=0 and j<=ncols-1:
                        if (graphmap[iw,j]!=2) and (graphmap[iw,j]!=3) and (graphmap[iw,j]!=5):
                            graphmap[iw,j] = 4
                            if distanceFromStart[iw,j] >= min_dist+1:
                                distanceFromStart[iw,j] = min_dist+1
                            parent[iw,j] = current

    route_idx = []
    route_idx.append(dest_node)
    
    # get path for route
    parent_dest = int(parent[iend,jend])
    idx_flat = parent_dest
    route_idx.append(idx_flat)

    while route_idx[-1] != start_node:
        parent_grid = np.unravel_index(idx_flat,(nrows,ncols))
        idx_flat = int(parent[parent_grid])
        route_idx.append(idx_flat)
    
    # convert flattened indices to metric coordinates
    route_length = len(route_idx)
    route = np.zeros((route_length+2,2))
    route[0,:] = start[0], start[1]
    route[1,:] = x0, y0
    route[-1,:] = goal[0], goal[1]
    
    route_flip = route_idx[::-1]

    for r in range(0,route_length,1):
        xy = np.unravel_index(route_flip[r],(nrows,ncols))
        route[r+1,0] = ((xy[1]+0.5)*x_spacing)
        route[r+1,1] = ((xy[0]+0.5)*y_spacing)
    
    return route


def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    main()

