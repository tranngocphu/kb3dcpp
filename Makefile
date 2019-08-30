srcs = Util.cpp Geodesic.cpp CD3D.cpp KB3D.cpp CDR.cpp LoS.cpp main.cpp 

objs = Util.o Geodesic.o CD3D.o KB3D.o CDR.o LoS.o main.o

kb3d : $(objs)
	g++ -o kb3d $(objs)

depend:
	makedepend $(srcs)

# DO NOT DELETE

Util.o: Util.h
Geodesic.o: Util.h Geodesic.h
CD3D.o: Util.h CD3D.h
KB3D.o: Util.h CD3D.h KB3D.h
CDR.o: Util.h CD3D.h KB3D.h CDR.h Geodesic.h
LoS.o: KB3D.h
main.o: Util.h CD3D.h KB3D.h CDR.h