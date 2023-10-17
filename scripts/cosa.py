#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import time

last_msg = 0
last_point = [0, 0] 
init_point = [0, 0]

x_0= 0
y_0=0

def callback(msg):
    print("Introducir coordenada")
    global last_msg

    last_msg = msg

global x,y,z
def callback2(msg): 
    global last_point
    x_1=msg.point.x
    y_1=msg.point.y
    rospy.loginfo("coordinates 1:x=%f y=%f" %(x_1,y_1))
    last_point = [x_1, y_1]
    

if __name__ == '__main__':
    rospy.init_node("path_generation")
    rospy.Subscriber("/hector_exploration_node/global_costmap/costmap", OccupancyGrid, callback)
    rospy.Subscriber("/clicked_point", PointStamped, callback2)

    rospy.spin()


    init_point= [0., 0.]

    np.clip(last_point[0], a_min = -50, a_max = 50)
    np.clip(last_point[1], a_min = -50, a_max = 50)

    last_msg.data = list(last_msg.data)

   
    
    

    # size = 5
    # for y in range(-size, size):
    #     for x in range(-size, size):
    #         last_msg.data[((y_mat + y) * last_msg.info.width) + (x_mat + x)] = 35
    

    # Empty list
    res = []
    N = 2000
    R = 900

    for idy in np.linspace(0, last_msg.info.height - 1, N, dtype=int):
        row = []
        for idx in np.linspace(0, last_msg.info.width - 1, N, dtype=int):
            # Getting incremented chunks
            row.append(last_msg.data[((last_msg.info.height - idy - 1) * last_msg.info.width) + idx])
        res.append(row)

    x_mat=int(((last_point[0]+50)/100)*N)
    y_mat=int(((last_point[1]+50)/100)*N)
    last_point_mat= [x_mat, N - y_mat]
    # last_point_mat= [460, 475]

    print(init_point)
    print((init_point[0]+50)/100)
    x_mat=int(((init_point[0]+50)/100)*N)
    y_mat=int(((init_point[1]+50)/100)*N)
    init_point_mat= [x_mat, y_mat]
   
    # last_point = [x_mat, y_mat]
    global_min = 100000

    print(last_point_mat)
    print(init_point_mat)

    found_points = set()
    found_points.add(tuple(init_point_mat))
    traveled_points = set()
    hijos_padres = {
        tuple(init_point_mat): (-1, -1)
    }
    last_found = 0

    #print(res)

    start = time.time( )
    for f in range(50000):
        traveled_points.add(tuple(init_point_mat))
        found_points.remove(tuple(init_point_mat))
        # print("punto de inicio:", init_point_mat)

        if np.sqrt((last_point_mat[0] - init_point_mat[0])**2 + (last_point_mat[1] - init_point_mat[1])**2) < 15:
            break

        last_found = init_point_mat

        search_step_size = 4

        vecinos = [
            [init_point_mat[0]+search_step_size, init_point_mat[1]],
            [init_point_mat[0]-search_step_size, init_point_mat[1]],
            [init_point_mat[0],   init_point_mat[1]+search_step_size],
            [init_point_mat[0],   init_point_mat[1]-search_step_size],

            [init_point_mat[0]+search_step_size, init_point_mat[1]+search_step_size],
            [init_point_mat[0]-search_step_size, init_point_mat[1]+search_step_size],
            [init_point_mat[0]+search_step_size,   init_point_mat[1]-search_step_size],
            [init_point_mat[0]-search_step_size,   init_point_mat[1]-search_step_size]
        ]

        dist_min=10000000000
        vecino_dist_min=0
        for i in vecinos:
            if res[i[1]][i[0]] < 70 and tuple(i) not in found_points and tuple(i) not in traveled_points:
                distancia = np.sqrt((last_point_mat[0] - i[0])**2 + (last_point_mat[1] - i[1])**2)
                # print(i, distancia)
                if distancia<dist_min:
                    dist_min=distancia
                    vecino_dist_min=i
                found_points.add(tuple(i))

            else:
                # print("vecino", i, "no es valido")
                pass

        # print("gaming", dist_min, vecino_dist_min)

        current_dist = np.sqrt((last_point_mat[0] - init_point_mat[0])**2 + (last_point_mat[1] - init_point_mat[1])**2)

        if dist_min < current_dist:
            hijos_padres[tuple(vecino_dist_min)] = tuple(init_point_mat)
            init_point_mat = vecino_dist_min

        else:
            min_found = 100000000
            min_curr_point = (0, 0)
            for point in found_points:
                dist = np.sqrt((last_point_mat[0] - point[0])**2 + (last_point_mat[1] - point[1])**2)
                if dist < min_found:
                    min_found = dist
                    min_curr_point = point

            hijos_padres[tuple(min_curr_point)] = tuple(init_point_mat)
            init_point_mat = min_curr_point

    print(time.time() - start)

    current_pos=tuple(last_found)
    puntos_a_seguir=[]
    while hijos_padres[current_pos] != (-1,-1):
        puntos_a_seguir.append(current_pos)
        current_pos=hijos_padres[current_pos]
    
    puntos_a_seguir = puntos_a_seguir[::-1]

    size = 1
    for i in puntos_a_seguir:
        for x in range(-size, size):
            for y in range(-size, size):
                res[i[1] + x][i[0] + y] = 80

    print(puntos_a_seguir[::-1])

    #print(hijos_padres)
    print(hijos_padres[tuple(last_found)])

    # last_msg = np.random.random((12, 12))
    #print(last_msg)
    plt.imshow(res)
    plt.show()