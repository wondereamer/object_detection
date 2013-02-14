include ../../makeconf

INCPATH = -I../../include

CFLAGS = $(MOREFLAGS) $(WARNLEVEL) -pedantic $(INCPATH)

SOURCES.C    =  jmesh.cpp

OBJECTS.O    =  $(SOURCES.C:.cpp=.o)

../OBJECTS/jmesh.o: $(OBJECTS.O)
	$(LD) -r -o ../OBJECTS/jmesh.o $(OBJECTS.O)

depend:
	makedepend -- $(CFLAGS) -- $(SOURCES.C)
	$(RM) -f Makefile.bak

clean :
	$(RM) -f *.o ../OBJECTS/jmesh.o

.cpp.o:
	$(CXX) $(CFLAGS) -c -o $@ $<

# DO NOT DELETE

jmesh.o: ../../include/jmesh.h ../../include/j_mesh.h /usr/include/stdio.h
jmesh.o: /usr/include/features.h /usr/include/libio.h
jmesh.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/math.h
jmesh.o: /usr/include/limits.h ../../include/list.h ../../include/point.h
jmesh.o: ../../include/vertex.h ../../include/edge.h ../../include/triangle.h
jmesh.o: ../../include/matrix.h ../../include/tin.h /usr/include/stdlib.h
jmesh.o: /usr/include/alloca.h /usr/include/string.h /usr/include/xlocale.h
