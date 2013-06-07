##
# @file server.py
# @brief draw all kind of data from c++ client
# @author Dignjie.Wang
# @version 0.1
# @date 2012-10-10

import matplotlib.pyplot as plt
import matplotlib.path as mpath
import numpy as np
import math
import copy


from SimpleXMLRPCServer import SimpleXMLRPCServer
#from m_util import log,Logger
#log.add_level(Logger.INFO)

def discrete_differential(y_list):
    '''docstring for discrete_differential''' 
    t = copy.deepcopy(y_list)
    for i,elem in enumerate(y_list[1:len(y_list)]):
        y_list[i+1] -= t[i]
    y_list[0] = 0
    return y_list

def previous_sum(vals):
    '''docstring for revious_sum''' 
    for i,elem in enumerate(vals[1:len(vals)]):
        vals[i+1] += vals[i]
    return vals

def angle_vector(x, y):
    '''docstring for angle_vector''' 
    t = math.degrees(math.atan2(y,x))
    return t

def plot_XY(y_list, x_list):
    '''docstring for plot_fun''' 
    #plt.plot(x_list,fun(x_lsit), 'bo', x_list, fun(x_list), 'k')
    #plt.plot(x_list, y_list, 'k')
    plt.plot(x_list, y_list, 'o-')
    plt.xlabel('X')
    plt.xlabel('Y')
    plt.show()

def draw_vectors_2D(x_list, y_list):
    '''docstring for force_with_spatial()''' 
    def x_domain(y_list):
        return range(1, len(y_list) + 1)

    #log.info("draw_vectors_2D is working...")
    strengths = []
    angles = []
    yticks = range(0,15)
    yticks += [100,200,300]
    max_ = 0
    i = 0;
    plt.subplot(211)    
    for x, y in zip(x_list, y_list):
        i += 1
        t = math.sqrt(x**2 + y**2)
        strengths.append(t)
        if t > max_:
           max_ = t 
        # draw direction
        plt.quiver(i,t,x,y)
        angles.append(angle_vector(x, y))

    #log.info(strengths,"force strengths:")
    X = range(1,len(strengths) + 1)    
    circle = mpath.Path.unit_circle()
    verts = np.concatenate([circle.vertices[::-1, ...]])
    codes = np.concatenate([circle.codes])
    circle_path = mpath.Path(verts, codes)
    plt.plot(X, strengths, '--r', marker=circle_path, markersize=5)
    #plt.axis([0, len(x_list) + 3, 0, len(x_list) + 3])
    plt.xticks(range(0, len(x_list) + 3))
    #plt.yticks(yticks)
    plt.xlabel("spatial distance")
    plt.ylabel("force strength" )

    plt.subplot(212) 
    discrete_differential(angles)
    X = x_domain(angles)
    plt.plot(X, angles, '--r', marker=circle_path, markersize=5)
    #plt.axis([0, len(X) + 3, 0, len(X) + 3])
    plt.xticks(range(0, len(X) + 3))
    plt.xlabel("spatial distance")
    plt.ylabel("direction change ratio" )
    plt.show()
    plt.savefig("one_pixel.jpg")
    return 0
def draw_force_field(x_list, y_list, width, height):
    '''docstring for draw_force_field''' 
    #log.info("draw_force_field is working..." )
    X = []
    Y = []
    for row in range(0,height):
        one_row_x = []
        one_row_y = []
        for col in range(0, width):
            i = row * width + col
            one_row_x.append(x_list[i])
            one_row_y.append(y_list[i])
        X.append(one_row_x)
        Y.append(one_row_y)
        #X.insert(0, one_row_x)
        #Y.insert(0, one_row_y)

    Q = plt.quiver( X, Y)
    plt.quiverkey(Q, 0.5, 0.92, 2, r'')
    #l,r,b,t = plt.axis()
    #dx, dy = r-l, t-b
    ##axis([l-0.05*dx, r+0.05*dx, b-0.05*dy, t+0.05*dy])
    #plt.xticks(range(0,width +3))
    #plt.yticks(range(0,height + 3))
    plt.title('Attraction Force field')
    plt.savefig("pixel.jpg")
    plt.show()
    return 0
if __name__ == '__main__':
    # A simple server with simple arithmetic functions
    #server = SimpleXMLRPCServer(("localhost", 8000))
    #print "Listening on port 8000..."
    #server.register_multicall_functions()
    #server.register_function(draw_vectors_2D, 'draw_vectors_2D')
    #server.register_function(draw_force_field, 'draw_force_field')
    #server.serve_forever()
    draw_vectors_2D([1,2],[3,4])

