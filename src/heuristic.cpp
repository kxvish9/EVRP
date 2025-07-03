#include "heuristic.hpp"
#include "EVRP.hpp"
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>      // For std::mt19937
#include <chrono>      // For seeding the random number generator
using namespace std;

// Global solution object, declared in heuristic.hpp
solution *best_sol;

// Global variables to maintain the state of the SA algorithm across multiple calls
static int* current_tour = nullptr;
static int current_tour_size = 0;
static double current_energy = DBL_MAX;
static double T = 1000.0; // Temperature

// --- Helper Functions ---
static void two_opt_swap(int* tour, int size, int i, int j) {
    std::reverse(tour + i, tour + j);
}

// Creates a valid, feasible initial tour to start the search
static void create_initial_solution() {
    best_sol->steps = 0;
    best_sol->tour[best_sol->steps++] = DEPOT;

    bool visited[NUM_OF_CUSTOMERS + 1] = {false};
    int customers_visited = 0;

    while (customers_visited < NUM_OF_CUSTOMERS) {
        int last_node = best_sol->tour[best_sol->steps - 1];
        int next_node = -1;
        double min_dist = DBL_MAX;

        for (int i = 1; i <= NUM_OF_CUSTOMERS; i++) {
            if (!visited[i]) {
                double dist = get_distance(last_node, i);
                if (dist < min_dist) {
                    min_dist = dist;
                    next_node = i;
                }
            }
        }

        if (next_node != -1) {
            best_sol->tour[best_sol->steps++] = next_node;
            visited[next_node] = true;
            customers_visited++;
        } else {
            break;
        }
    }
    best_sol->tour[best_sol->steps++] = DEPOT;
    best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);

    if (best_sol->tour_length >= DBL_MAX) {
        cout << "Error: Initial greedy solution is infeasible. Using random shuffle fallback." << endl;
        for(int i=0; i<NUM_OF_CUSTOMERS; i++) best_sol->tour[i+1] = i+1;
        
        // This is the updated, modern way to shuffle
        unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        std::mt19937 g(seed);
        std::shuffle(&best_sol->tour[1], &best_sol->tour[NUM_OF_CUSTOMERS+1], g);

        best_sol->steps = NUM_OF_CUSTOMERS + 2;
        best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);
    }
}

// --- Main Heuristic Functions ---

void initialize_heuristic() {
    best_sol = new solution;
    best_sol->tour = new int[ACTUAL_PROBLEM_SIZE * 2];

    create_initial_solution();

    // Initialize the state for the current run
    current_tour_size = best_sol->steps;
    current_tour = new int[current_tour_size];
    std::copy(best_sol->tour, best_sol->tour + current_tour_size, current_tour);
    current_energy = best_sol->tour_length;
    
    // Reset temperature for the start of a new run
    T = 1000.0;
}

void run_heuristic() {
    // This function now performs a small batch of SA iterations each time it's called.
    
    double cooling_rate = 0.9995;
    int iterations_per_call = 100; // Perform 100 swaps per call to run_heuristic

    for (int i = 0; i < iterations_per_call; ++i) {
        int* neighbor_tour = new int[current_tour_size];
        std::copy(current_tour, current_tour + current_tour_size, neighbor_tour);
        
        // Generate two random, distinct indices for the 2-Opt swap
        int idx1 = 1 + rand() % (current_tour_size - 2);
        int idx2 = 1 + rand() % (current_tour_size - 2);

        if (abs(idx1 - idx2) < 2) {
             delete[] neighbor_tour;
             continue;
        }
        if (idx1 > idx2) std::swap(idx1, idx2);
        
        two_opt_swap(neighbor_tour, current_tour_size, idx1, idx2);
        double neighbor_energy = fitness_evaluation(neighbor_tour, current_tour_size);

        if (neighbor_energy < DBL_MAX) {
            if (neighbor_energy < current_energy) {
                std::copy(neighbor_tour, neighbor_tour + current_tour_size, current_tour);
                current_energy = neighbor_energy;
            } else {
                double acceptance_prob = exp((current_energy - neighbor_energy) / T);
                if ((double)rand() / RAND_MAX < acceptance_prob) {
                    std::copy(neighbor_tour, neighbor_tour + current_tour_size, current_tour);
                    current_energy = neighbor_energy;
                }
            }
        }
        
        delete[] neighbor_tour;

        if (current_energy < best_sol->tour_length) {
            best_sol->tour_length = current_energy;
            std::copy(current_tour, current_tour + current_tour_size, best_sol->tour);
            best_sol->steps = current_tour_size;
        }
    }

    // Cool the temperature down slightly on each call
    T *= cooling_rate;
}


void free_heuristic() {
    delete[] best_sol->tour;
    delete best_sol;
    delete[] current_tour; // Free the memory for the current tour state
    current_tour = nullptr;
}