#!/usr/bin/python                       
# remove extral points that not inside mesh from file
import fileinput                         
# index to points referenced by face
indexes = set()
faces = []
for line in fileinput.input(inplace = False):
    line = line.strip(" \n")
    strList = line.split(" ")
    if len(strList) == 4:
        indexes.add(int(strList[1]))
        indexes.add(int(strList[2]))
        indexes.add(int(strList[3]))
        faces.append(line)

i_old2new = { }
points = { }
for line in fileinput.input(inplace = False):
    if fileinput.lineno() == 2:
        continue
    line = line.strip(" \n")
    strList = line.split(" ")
    if len(strList) == 3:
        # i is the old index
        i = fileinput.lineno() - 2 - 1
        if i in indexes:
            # the points is in the face
            # map old index to new index
            i_old2new[i] = points.setdefault(line, len(points))

f = open('mds_rst.off', 'w')
# output the header of OFF file
print >> f, "OFF"
print >> f, str(len(points))  + " " + str(len(faces)) + " 0" 
# output points in new index order
for key, value in sorted(points.iteritems(), key = lambda (k,v): (v,k)):
    print >> f, key
# output faces
for face in faces:
    strList = face.split(" ")
    print >> f, "3 %s %s %s" % (i_old2new[int(strList[1])], i_old2new[int(strList[2])], i_old2new[int(strList[3])])
