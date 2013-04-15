
from pylab import *

# Plot circle or radius 3
an = linspace(0,pi/180*120,50)

for i in [0,30,60,90,120]:
    # code...
    plot([0,0.2*cos(pi/180*i)],[0, 0.2*sin(pi/180*i)],c = "black")

#subplot(224)
for i in range(0,11):
    #ax.plot( *xy(0.02*i,phis), c='black',ls='-' )
    r = 0.02*i
    plot( r*cos(an), r*sin(an) ,c = 'black' )
axis('equal')
#axis([-3,3,-3,3])
title('circle',fontsize=10)

show()
