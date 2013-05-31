import matplotlib.pyplot as plt
from matplotlib.offsetbox import TextArea, DrawingArea, OffsetImage, \
     AnnotationBbox

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

for line in fileinput.input(inplace = False):
    mark.add(fileinput.filename())
    if len(mark) == 1:
       source.append(line)
    else:
       target.append(line)

temp = []
average = []
all_average = []
for i, line in enumerate(source):
    line = line.rstrip("\n")
    if line.startswith('--'):
        average.append(temp[1])
    elif line.startswith('##'):
        print "average" 
        all_average.append()
    else:
        temp.append(line)

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
    xy = (0.5, 0.7)
    offsetbox = TextArea("Test", minimumdescent=False)

    ab = AnnotationBbox(offsetbox, xy,
                        xybox=(1.02, xy[1]),
                        xycoords='data',
                        boxcoords=("axes fraction", "data"),
                        box_alignment=(0.,0.5),
                        arrowprops=dict(arrowstyle="->"))
    plt.plot(x_list, y_list, 'o--')
    plt.xlabel('X')
    plt.xticks(range(len(fnames)), fnames, size='small')
    plt.xlabel('Y')
    ax.add_artist(ab)






    plt.show()

def plot2XY(y_list, x_list, avS, y_list2, x_list2, avT):
    '''docstring for plot_fun''' 
    #plt.plot(x_list,fun(x_lsit), 'bo', x_list, fun(x_list), 'k')
    #plt.plot(x_list, y_list, 'k')


    fig = plt.gcf()
    fig.clf()
    ax = plt.subplot(111)


    plt.plot(x_list, y_list, 'o-', color = 'red' )
    plt.plot(x_list2, y_list2, 'o--', color = 'blue' )
    plt.plot(x_list2, [avS]*len(x_list2), 'k', color = 'red' )
    plt.plot(x_list, [avT]*len(x_list), 'k', color = 'blue' )
    plt.xlabel('X')
    plt.xlabel('Y')
    leg =  ax.legend(('Model length', 'Data length'),
                       'upper center', shadow = True)

    frame  =  leg.get_frame()
    frame.set_facecolor('0.80')    # set the frame face color to light gray

    # matplotlib.text.Text instances
    for t in leg.get_texts():
        t.set_fontsize('small')    # the legend text fontsize

    # matplotlib.lines.Line2D instances
    for l in leg.get_lines():
        l.set_linewidth(1.5)  # the legend line width

    #xy = (0.5, 0.7)
    #offsetbox = TextArea("Test", minimumdescent=False)

    #ab = AnnotationBbox(offsetbox, xy,
                        #xybox=(0.02, 0.3),
                        #xycoords='data',
                        ##boxcoords=("axes fraction", "data"),
                        #box_alignment=(0.,0.5),
                            #boxcoords="offset points",
                        #arrowprops=dict(arrowstyle="->"))
    #ax.add_artist(ab)
    #plt.xticks(range(0, 15))
    plt.draw()
    plt.show()

if __name__ == '__main__':
    if len(mark) == 1:
        plot_XY(valuesS, range(0,len(valuesS)), fnamesS)
    elif len(mark) == 2:
        plot2XY(valuesS, range(0,len(valuesS)), avS, valuesT, 
                range(0, len(valuesT)), avT)
    else:
        assert(False)
