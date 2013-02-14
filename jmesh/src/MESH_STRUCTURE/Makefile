include ../../makeconf

INCPATH = -I../../include
CFLAGS = $(MOREFLAGS) $(WARNLEVEL) -pedantic $(INCPATH) -I../RANGECODER

SOURCES.C    =  point.cpp\
		vertex.cpp\
		edge.cpp\
		triangle.cpp\
		tin.cpp\
		io.cpp\
		checkAndRepair.cpp
		
OBJECTS.O    =  $(SOURCES.C:.cpp=.o)

../OBJECTS/mesh_structure.o: $(OBJECTS.O)
	$(LD) -r -o ../OBJECTS/mesh_structure.o $(OBJECTS.O)

depend:
	makedepend -- $(CFLAGS) -- $(SOURCES.C)
	$(RM) -f Makefile.bak

clean :
	$(RM) -f *.o ../OBJECTS/mesh_structure.o

.cpp.o:
	$(CXX) $(CFLAGS) -c -o $@ $<

# DO NOT DELETE

point.o: ../../include/point.h ../../include/j_mesh.h /usr/include/stdio.h
point.o: /usr/include/features.h /usr/include/libio.h
point.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/math.h
point.o: /usr/include/limits.h /usr/include/stdlib.h /usr/include/alloca.h
point.o: /usr/include/errno.h
vertex.o: ../../include/vertex.h ../../include/j_mesh.h /usr/include/stdio.h
vertex.o: /usr/include/features.h /usr/include/libio.h
vertex.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/math.h
vertex.o: /usr/include/limits.h ../../include/list.h ../../include/point.h
vertex.o: ../../include/edge.h ../../include/triangle.h ../../include/jmesh.h
vertex.o: ../../include/matrix.h ../../include/tin.h /usr/include/stdlib.h
vertex.o: /usr/include/alloca.h /usr/include/errno.h
edge.o: ../../include/edge.h ../../include/j_mesh.h /usr/include/stdio.h
edge.o: /usr/include/features.h /usr/include/libio.h /usr/include/_G_config.h
edge.o: /usr/include/wchar.h /usr/include/math.h /usr/include/limits.h
edge.o: ../../include/vertex.h ../../include/list.h ../../include/point.h
edge.o: ../../include/triangle.h ../../include/jmesh.h ../../include/matrix.h
edge.o: ../../include/tin.h
triangle.o: ../../include/triangle.h ../../include/jmesh.h
triangle.o: ../../include/j_mesh.h /usr/include/stdio.h
triangle.o: /usr/include/features.h /usr/include/libio.h
triangle.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/math.h
triangle.o: /usr/include/limits.h ../../include/list.h ../../include/point.h
triangle.o: ../../include/vertex.h ../../include/edge.h
triangle.o: ../../include/matrix.h ../../include/tin.h /usr/include/stdlib.h
triangle.o: /usr/include/alloca.h
tin.o: ../../include/tin.h ../../include/jmesh.h ../../include/j_mesh.h
tin.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
tin.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/math.h
tin.o: /usr/include/limits.h ../../include/list.h ../../include/point.h
tin.o: ../../include/vertex.h ../../include/edge.h ../../include/triangle.h
tin.o: ../../include/matrix.h /usr/include/stdlib.h /usr/include/alloca.h
tin.o: /usr/include/string.h /usr/include/xlocale.h
io.o: ../../include/jmesh.h ../../include/j_mesh.h /usr/include/stdio.h
io.o: /usr/include/features.h /usr/include/libio.h /usr/include/_G_config.h
io.o: /usr/include/wchar.h /usr/include/math.h /usr/include/limits.h
io.o: ../../include/list.h ../../include/point.h ../../include/vertex.h
io.o: ../../include/edge.h ../../include/triangle.h ../../include/matrix.h
io.o: ../../include/tin.h /usr/include/stdlib.h /usr/include/alloca.h
io.o: /usr/include/string.h /usr/include/xlocale.h /usr/include/ctype.h
io.o: /usr/include/endian.h
checkAndRepair.o: ../../include/jmesh.h ../../include/j_mesh.h
checkAndRepair.o: /usr/include/stdio.h /usr/include/features.h
checkAndRepair.o: /usr/include/libio.h /usr/include/_G_config.h
checkAndRepair.o: /usr/include/wchar.h /usr/include/math.h
checkAndRepair.o: /usr/include/limits.h ../../include/list.h
checkAndRepair.o: ../../include/point.h ../../include/vertex.h
checkAndRepair.o: ../../include/edge.h ../../include/triangle.h
checkAndRepair.o: ../../include/matrix.h ../../include/tin.h
checkAndRepair.o: ../../include/jqsort.h /usr/include/stdlib.h
checkAndRepair.o: /usr/include/alloca.h /usr/include/string.h
checkAndRepair.o: /usr/include/xlocale.h
