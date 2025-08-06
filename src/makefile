# Compiler to use
CXX = g++

# --- Compiler Flags ---
# Release flags: High optimization, target native machine architecture
CXXFLAGS_RELEASE = -O3 -march=native

# Debug flags: No optimization, include debug symbols for use with a debugger (like GDB)
CXXFLAGS_DEBUG = -O0 -g

# --- Target Names ---
TARGET_RELEASE = main
TARGET_DEBUG = main_debug

# --- Object Files (used by both builds) ---
OBJS = main.o heuristic.o EVRP.o stats.o config.o

# --- Build Rules ---

# Default rule if you just type "make": build the fast release version
all: release

# Rule to build the fast, optimized version
release: CXXFLAGS=$(CXXFLAGS_RELEASE)
release: $(OBJS)
	$(CXX) $^ -o $(TARGET_RELEASE)

# Rule to build the slow, debug-ready version
debug: CXXFLAGS=$(CXXFLAGS_DEBUG)
debug: $(OBJS)
	$(CXX) $^ -o $(TARGET_DEBUG)


# --- Compilation Rules for individual files ---
# These rules will use whichever CXXFLAGS is set by the target (release or debug)
main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp

heuristic.o: heuristic.cpp
	$(CXX) $(CXXFLAGS) -c heuristic.cpp

EVRP.o: EVRP.cpp
	$(CXX) $(CXXFLAGS) -c EVRP.cpp

config.o: config.cpp
	$(CXX) $(CXXFLAGS) -c config.cpp

stats.o: stats.cpp
	$(CXX) $(CXXFLAGS) -c stats.cpp


# --- Clean Rule ---
# This will remove all compiled files for both versions
clean:
	del /f /q *.o $(TARGET_RELEASE).exe $(TARGET_DEBUG).exe