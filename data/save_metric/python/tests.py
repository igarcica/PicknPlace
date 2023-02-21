import numpy as np
import open3d as o3d

can_obj_file = './o1_gr_can_seg.pcd'


def create_canonical():
    #Extract object size from point cloud
    #Having px/cm, distance to camera and object size compute new size and number of points
    #Create a rectangle with depth mean and new size (X,Y) with center the camera

    #X, Y?
    syn_can_matrix = []
    syn_can_x = []
    syn_can_y = []
    syn_can_depth = []

    # syn_can_array = np.ones((670,3))*0.40
    # print(syn_can_array)
    # print(syn_can_array.shape)

    syn_can_depth = np.array(np.ones(8)*0.40)
    # print(syn_can_depth)
    # print(len(syn_can_depth))

    syn_can_x = np.arange(-6,4,1)
    syn_can_y = np.arange(-2,10,1)
    print(len(syn_can_x))
    # print(syn_can_x)
    print(len(syn_can_y))
    # print(syn_can_y)

    syn_can_matrix=[0,0,0]
    for i in range(0,len(syn_can_x)):
        for n in range(0,len(syn_can_y)):
            row = [syn_can_x[i],syn_can_y[n],0.4]
            syn_can_matrix = np.vstack([syn_can_matrix, row])
            #print(syn_can_matrix)
    print(syn_can_matrix)
    
    print(syn_can_matrix.shape)


    # syn_can_matrix = [syn_can_x[0], syn_can_y[0], syn_can_depth[0]]
    # for i in range(1,len(syn_can_x)):
    #     row = [syn_can_x[i], syn_can_y[i], syn_can_depth[i]]
    #     print(row)
    #     syn_can_matrix = np.vstack([syn_can_matrix, row])
    # print(syn_can_matrix)


can_pcd = o3d.io.read_point_cloud(can_obj_file)
can_data = np.asarray(can_pcd.points)
create_canonical()
