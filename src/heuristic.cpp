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
    std::reverse(tour + i, tour + j + 1);
}

// Creates a valid, feasible initial tour to start the search
static void create_initial_solution() {
    // Start at the depot
    best_sol->tour[0] = DEPOT;
    best_sol->steps = 1;

    // Create a list of all customer IDs (from 2 to NUM_OF_CUSTOMERS + 1)
    int customers[NUM_OF_CUSTOMERS];
    for (int i = 0; i < NUM_OF_CUSTOMERS; i++) {
        customers[i] = i + 2;
    }

    // Shuffle the customer list to create a random but complete tour
    unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 g(seed);
    std::shuffle(customers, customers + NUM_OF_CUSTOMERS, g);
    
    // Append the shuffled customers to the tour
    for (int i = 0; i < NUM_OF_CUSTOMERS; i++) {
        best_sol->tour[best_sol->steps++] = customers[i];
    }
    
    // End the tour at the depot
    best_sol->tour[best_sol->steps++] = DEPOT;
    
    // Evaluate the complete tour's fitness
    best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);

    // This check is a safeguard for problem instances with very tight constraints
    if (best_sol->tour_length >= DBL_MAX) {
        cout << "CRITICAL WARNING: A full random tour is infeasible. Problem constraints may be too tight." << endl;
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

        // Ensure the indices are different. Keep trying until they are.
        while (idx1 == idx2) {
            idx2 = 1 + rand() % (current_tour_size - 2);
        }

        // Ensure idx1 is smaller than idx2
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