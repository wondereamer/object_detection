import matplotlib.pyplot as plt
import numpy as np
#                   r   c    b     te    ao
matrix = np.array([[0.1, 0.5, 0.45, 0.55, 0.38],
                   [1,   0.13, 0.8,  0.3, 0.7],
                   [1, 1,   0.2,    0.7, 0.45],
                   [1, 1, 1,      0.17,   0.6],
                   [1, 1, 1,      1,   0.18]])
#matrix = np.array([[0, 0.25, 0.4, 0.7, 0.48, 0.8],
                   #[1, 0, 0.35, 0.65, 0.73, 0.9],
                   #[1, 1,   0,  0.7, 0.75, 0.6],
                   #[1, 1, 1,   0, 0.3, 0.55],
                   #[1, 1, 1,   1, 0, 0.42],
                   #[1, 1, 1,   1, 1, 0]])

#matrix = np.array([[0, 0.15, 0.2, 0.7, 0.68, 0.8],
                   #[1, 0, 0.25, 0.65, 0.73, 0.9],
                   #[1, 1,   0,  0.7, 0.75, 0.7],
                   #[1, 1, 1,   0, 0.22, 0.35],
                   #[1, 1, 1,   1, 0, 0.30],
                   #[1, 1, 1,   1, 1, 0]])
#matrix = np.array([[0, 0.5, 0.6],
                   #[1, 0, 0.25],
                   #[1, 1, 0]])
txt = None
pfig = plt.figure(1)
plt.imshow(matrix, interpolation = 'nearest',cmap = plt.cm.Greys_r, vmin = 0, vmax = 1)
##imshow(A, interpolation = 'nearest',cmap =  cm.Greys_r)
##imshow(A, interpolation = 'nearest')
##grid(True)
plt.colorbar()
plt.xticks([])
plt.yticks([])
plt.show()
#*************************************************************************
#def my_imshow(my_img,ax=None,**kwargs):
    #if ax is None:
        #ax = gca()
    #def format_coord(x, y):
        #x = int(x + 0.5)
        #y = int(y + 0.5)
        #try:
            #return "%s @ [%4i, %4i]" % (my_img[y, x], x, y)
        #except IndexError:
            #return ""
    #ax.imshow(my_img,**kwargs)
    #ax.format_coord = format_coord
    #show()

#my_imshow(A, interpolation = 'nearest' )

#*************************************************************************
#from matplotlib import pyplot as plt
#import numpy as np

#im = plt.imshow(np.random.rand(10,10)*255, interpolation='nearest')
#fig = plt.gcf()
#ax = plt.gca()

#class EventHandler:
    #def __init__(self):
        #fig.canvas.mpl_connect('button_press_event', self.onpress)

    #def onpress(self, event):
        #if event.inaxes!=ax:
            #return
        #xi, yi = (int(round(n)) for n in (event.xdata, event.ydata))
        #value = im.get_array()[xi,yi]
        #color = im.cmap(im.norm(value))
        #print xi,yi,value,color

#handler = EventHandler()

#plt.show()

#*************************************************************************

#def onclick(event):
    #global txt
    #txt = plt.text(event.xdata, event.ydata, 'TESTTEST', fontsize=8)
    #fig.canvas.draw()

#def offclick(event):
    ##txt.remove()
    ##fig.canvas.draw()
    #pass

#fig.canvas.mpl_connect('button_press_event', onclick)
#fig.canvas.mpl_connect('button_release_event', offclick) 

#plt.show()
