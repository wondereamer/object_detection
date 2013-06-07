import matplotlib.pyplot as plt
from matplotlib.offsetbox import TextArea, DrawingArea, OffsetImage, \
     AnnotationBbox
from pprint import pprint

def plot_rst(xList, yList, fnameList):
    '''docstring for plot_rst()''' 
    fig = plt.gcf()
    fig.clf()
    ax =  plt.subplot2grid((5,1),(0, 0),rowspan = 4)
    #ax = plt.subplot(111)


    xy = (0.5, 0.7)



    offsetbox = TextArea("Test", minimumdescent=False)

    ab = AnnotationBbox(offsetbox, xy,
                        xybox=(1.02, xy[1]),
                        xycoords='data',
                        boxcoords=("axes fraction", "data"),
                        box_alignment=(0.,0.5),
                        arrowprops=dict(arrowstyle="->"))
    ax.add_artist(ab)


    from matplotlib.patches import Circle
    da = DrawingArea(20, 20, 0, 0)
    p = Circle((10, 10), 10)
    da.add_artist(p)

    xy = [0.3, 0.55]
    ab = AnnotationBbox(da, xy,
                        xybox=(1.02, xy[1]),
                        xycoords='data',
                        boxcoords=("axes fraction", "data"),
                        box_alignment=(0.,0.5),
                        arrowprops=dict(arrowstyle="->"))
                        #arrowprops=None)

    ax.add_artist(ab)




    # another image


    from matplotlib._png import read_png
    #fn = get_sample_data("./61.png", asfileobj=False)
    arr_lena = read_png("./61.png")
    imagebox = OffsetImage(arr_lena, zoom=0.2)
    xy = (0.1, 0.1)
    print fnameList
    for i in range(0,len(fnameList)):
        ax.add_artist(AnnotationBbox(imagebox, xy,
                            xybox=(0.1 + i*0.2, -0.15),
                            xycoords='data',
                            boxcoords=("axes fraction", "data"),
                            #boxcoords="offset points",
                            pad=0.1,
                            arrowprops=dict(arrowstyle="->",
                                            connectionstyle="angle,angleA=0,angleB=90,rad=3")
                            ))

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)


    plt.draw()
    plt.show()

import fileinput                         
import re
# source file
source = []
# target file
target = []
fnamesT = []
valuesT = []
avT = 0
fnamesS = []
valuesS = []
avS = 0
mark = set()
overlap = False
cmp_gd = []
cmp_av = []
def read_log():
    '''docstring for read_log''' 
    import argparse
    parser =  argparse.ArgumentParser(description = 'Process some integers.')
    parser.add_argument('f', help = 'file name')
    parser.add_argument('o', help = 'if overlap')
    args = parser.parse_args()
    t_length = 20
    global overlap
    if args.o == 'y':
        overlap = True 
    else:
        overlap = False
    f = open(args.f, "r")
    for line in f.readlines():
       source.append(line)

    temp = []
    ylist = []
    fname = []
    # map from source file to (ylist, average)
    all_average = { }
    global cmp_gd
    for i, line in enumerate(source):
        line = line.rstrip("\n")
        if line.startswith('--'):
            if len(temp) >= 1:
                fname.append(temp[0])
                try:
                    ylist.append(float(temp[1]))
                except Exception, e:
                    # none comparable item,
                    # this would make average estimation unreliable
                    assert False
                    ylist.append(1500)
            temp = []
        elif line.startswith('##'):
            all_average[temp.pop()] = (ylist, temp[0], fname)
            gl = len(ylist)/t_length
            b = 0
            e = gl
            gd = []
            av = []
            while True:
                g = ylist[b:e]
                if not g:
                   break
                m = min(g)
                for i,v in enumerate(g):
                    if v == m and v < 0.01:
                        # remove self comparing
                        g.pop(i)
                        break
                # the average distance to a group
                av.append(sum(g) / float(len(g)))
                # the minimum distance to a group
                gd.append(min(g))
                b = e
                e += gl
            print gl, "-------------------", len(gd)
            cmp_gd.append(gd)
            cmp_av.append(av)
            temp = []
            ylist = []
            fname = []
        else:
            temp.append(line)
    #pprint(cmp_gd)

    sumv = 0.0
    if cmp_gd:
        for i,v in enumerate(cmp_gd[0]):
            col = []
            for r in cmp_gd[1:]:
                col.append(r[i])
            if v < min(col):
                sumv += 1

    sumav = 0.0
    if cmp_av:
        for i,v in enumerate(cmp_av[0]):
            col = []
            for r in cmp_av[1:]:
                col.append(r[i])
            if v < min(col):
                sumav+= 1
    print "*****************************"
    print "minimum distance:" 
    print sumv
    print sumv/t_length
    print "average distance:"
    print sumav
    print sumav/t_length
    print "*****************************"

    return all_average

    #if i < len(source) - 2:
       #if re.findall("[a-zA-Z]+", line):
           #fnamesS.append(line)
       #elif re.findall("[0-9]+", line):
           #valuesS.append(float(line))
    #elif i == len(source) - 1:
       #avS = float(line)


def plot_XY(y_list, x_list, fnames):
    '''docstring for plot_fun''' 
    #plt.plot(x_list,fun(x_lsit), 'bo', x_list, fun(x_list), 'k')
    #plt.plot(x_list, y_list, 'k')

    fig = plt.gcf()
    fig.clf()
    ax = plt.subplot(111)
    ax.plot(x_list, y_list, 'o--')
    plt.xlabel('X')
    plt.xticks(range(len(fnames)), fnames, size='small')
    plt.xlabel('Y')

    #ax.set_xlim(1, 10)
    #ax.set_ylim(1, 5)

    plt.show()



def plot_multi_xy(log_data, overlap):
    '''docstring for plot_fun''' 
    #plt.plot(x_list,fun(x_lsit), 'bo', x_list, fun(x_list), 'k')
    #plt.plot(x_list, y_list, 'k')
    #plt.plot(x_list2, y_list2, 'o--', color = 'blue' )
    fig = plt.figure(1,figsize=(8,5))
    ax = fig.add_subplot(111)
    b = 1
    i = 0
    for key, value in log_data.iteritems():
        e =  b + len(value[0])
        if overlap:
            x_list = range(0, 400)
        else:
            x_list = range(b, e)
        av = float(value[1])
        if i % 2:
            ax.plot(x_list, value[0], 'o--', color = 'red' )
        else:
            ax.plot(x_list, value[0], 'o--', color = 'blue' )

        min_y = min(value[0])
        av_x = b + (e - b)/2
        ax.annotate(key, xy=(av_x,  min_y),  xycoords='data',
                    xytext=(-10, -10), textcoords='offset points',
                    bbox=dict(boxstyle="round", fc="0.8"),
                    )
        i += 1
        b = e
        
    #plt.xlabel('X')
    #plt.xlabel('Y')
    # legend

    leg =  ax.legend((u'Distance', u'Average Distance',u'Distance', u'Average Distance'),
                       'upper center', shadow = True)

    frame  =  leg.get_frame()
    frame.set_facecolor('0.80')    # set the frame face color to light gray

    for t in leg.get_texts():
        t.set_fontsize('small')    # the legend text fontsize

    for l in leg.get_lines():
        l.set_linewidth(1.5)  # the legend line width

    #from matplotlib._png import read_png
    #arr_lena = read_png("./61.png")
    #imagebox = OffsetImage(arr_lena, zoom=0.2)
    #ax.add_artist(AnnotationBbox(imagebox, [30,500],
                        #xybox=(180, -10),
                        #xycoords='data',
                        ##boxcoords=("axes fraction", "data"),
                        #boxcoords="offset points",
                        #pad=0.1,
                        #arrowprops=dict(arrowstyle="->",
                                        #connectionstyle="angle,angleA=0,angleB=90,rad=3")
                        #))



    #plt.xticks(range(0, 15))
    plt.show()

from matplotlib.lines import Line2D
import numpy as np
def plot_shape(xList, yList):
    import fileinput                         
    fig = plt.figure(1,figsize=(8,5))
    ax = fig.add_subplot(111)
    lines0 = [[],[],[],[],[],[],[]]
    lines1 = [[],[],[],[],[],[],[]]
    lines2 = [[],[],[],[],[],[],[]]
    for line in fileinput.input(inplace = False):
        line = line.rstrip('\r\n')
        line_list = line.split('\t')
        assert len(line_list) == 7
        for i,col in enumerate(line_list):
            # code...
            col2 = col.split(';')
            assert len(col2) == 3
            lines0[i].append(float(col2[0])+2)
            lines1[i].append(float(col2[1])+2)
            lines2[i].append(float(col2[2])+2)

    t = np.arange(0.0, 1.0, 0.1)
    s = np.sin(2*np.pi*t)
    linestyles = ['_', '-', '--', ':']
    markers = []
    for m in Line2D.markers:
        try:
            if len(m) == 1 and m != ' ':
                markers.append(m)
        except TypeError:
            pass


    styles = markers + [
    r'$\lambda$',
    r'$\bowtie$',
    r'$\circlearrowleft$',
    r'$\clubsuit$',
    r'$\checkmark$']

    colors = ('b', 'g', 'r', 'c', 'm', 'y', 'k')

    #plt.plot(t, s, linestyle='None', marker=styles[0], color=colors[0], markersize=10)
    #plt.plot(t, s, linestyle='None', marker=styles[1], color=colors[0], markersize=10)
    assert len(lines0[1]) == 37
    print lines0[1]
    plt.plot(range(0,37), lines0[0], linestyle = 'None' , marker=styles[0], color=colors[0], markersize=10)
    plt.plot(range(0,37), lines0[1], linestyle = 'None' , marker=styles[1], color=colors[1], markersize=10)
    plt.plot(range(0,37), lines0[2], linestyle = 'None' , marker=styles[4], color=colors[2], markersize=10)
    plt.plot(range(0,37), lines0[3], linestyle = 'None' , marker=styles[5], color=colors[3], markersize=10)
    plt.plot(range(0,37), lines0[4], linestyle = 'None' , marker=styles[6], color=colors[4], markersize=10)
    plt.plot(range(0,37), lines0[5], linestyle = 'None' , marker=styles[7], color=colors[5], markersize=10)
    plt.plot(range(0,37), lines0[6], linestyle = 'None' , marker=styles[8], color=colors[6], markersize=10)


    #plt.plot(range(0,37), lines1[0], linestyle = 'None' , marker=styles[0], color=colors[0], markersize=10)
    #plt.plot(range(0,37), lines1[1], linestyle = 'None' , marker=styles[1], color=colors[1], markersize=10)
    #plt.plot(range(0,37), lines1[2], linestyle = 'None' , marker=styles[4], color=colors[2], markersize=10)
    #plt.plot(range(0,37), lines1[3], linestyle = 'None' , marker=styles[5], color=colors[3], markersize=10)
    #plt.plot(range(0,37), lines1[4], linestyle = 'None' , marker=styles[6], color=colors[4], markersize=10)
    #plt.plot(range(0,37), lines1[5], linestyle = 'None' , marker=styles[7], color=colors[5], markersize=10)
    #plt.plot(range(0,37), lines1[6], linestyle = 'None' , marker=styles[8], color=colors[6], markersize=10)

    #plt.plot(range(0,37), lines2[0], linestyle = 'None'  , marker=styles[0], color=colors[0], markersize=10)
    #plt.plot(range(0,37), lines2[1], linestyle = 'None'  , marker=styles[1], color=colors[1], markersize=10)
    #plt.plot(range(0,37), lines2[2], linestyle = 'None'  , marker=styles[4], color=colors[2], markersize=10)
    #plt.plot(range(0,37), lines2[3], linestyle = 'None'  , marker=styles[5], color=colors[3], markersize=10)
    #plt.plot(range(0,37), lines2[4], linestyle = 'None'  , marker=styles[6], color=colors[4], markersize=10)
    #plt.plot(range(0,37), lines2[5], linestyle = 'None'  , marker=styles[7], color=colors[5], markersize=10)
    #plt.plot(range(0,37), lines2[6], linestyle = 'None'  , marker=styles[8], color=colors[6], markersize=10)

    leg =  ax.legend((u'1th Iteration', u'2th Iteration',u'3th Iteration',
        u'4th Iteration','5th Iteration','6th Iteration','7th Iteration' ),
                       'upper right', shadow = True)

    frame  =  leg.get_frame()
    frame.set_facecolor('0.80')    # set the frame face color to light gray

    for t in leg.get_texts():
        t.set_fontsize('small')    # the legend text fontsize

    for l in leg.get_lines():
        l.set_linewidth(1.5)  # the legend line width

    ax.set_ylim(0, 105)
    ax.set_xlim(0, 38)

    plt.xticks(range(38))
    plt.show()

if __name__ == '__main__':
   global overlap
   data = read_log()
   l = set()
   for v in data:
       l.add(len(v))
   if len(l) >= 2:
       overlap = False
   plot_multi_xy(data, overlap)
   ##av = read_log()
   ##value = av['cup'][0]
   ##fname = av['cup'][2]
   ##plot_XY(value, range(0, len(value)),fname)
