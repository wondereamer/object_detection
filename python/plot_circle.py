
from pylab import *

# Plot circle or radius 3
an = linspace(pi/180*60,pi/180*120,200)
# plot line
for i in [60,90,120]:
    # code...
    plot([0,0.2*cos(pi/180*i)],[0, 0.2*sin(pi/180*i)],c = "black")

#subplot(224)
# plot curve
r = 0.2
plot( r*cos(an), r*sin(an) ,c = 'black' )
axis('equal')
#axis([-3,3,-3,3])
title('circle',fontsize=10)

show()
