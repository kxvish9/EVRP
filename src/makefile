# Compiler to use
CXX = g++

# Compiler flags for a high-performance release build
CXXFLAGS = -O3 -march=native

# The name of your final program
TARGET = main

# --- Rules ---

all: $(TARGET)

$(TARGET): main.o heuristic.o EVRP.o stats.o
	$(CXX) $^ -o $(TARGET)

main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp

heuristic.o: heuristic.cpp
	$(CXX) $(CXXFLAGS) -c heuristic.cpp

EVRP.o: EVRP.cpp
	$(CXX) $(CXXFLAGS) -c EVRP.cpp

stats.o: stats.cpp
	$(CXX) $(CXXFLAGS) -c stats.cpp

clean:
	del /f /q *.o $(TARGET)