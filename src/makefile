all: main

main: main.o heuristic.o EVRP.o stats.o
	g++ -g main.o heuristic.o EVRP.o stats.o -o main

main.o: main.cpp
	g++ -g -c main.cpp

heuristic.o: heuristic.cpp
	g++ -g -c heuristic.cpp

EVRP.o: EVRP.cpp
	g++ -g -c EVRP.cpp

stats.o: stats.cpp
	g++ -g -c stats.cpp

clean:
	del *.o main.exe main