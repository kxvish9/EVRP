#include "heuristic.hpp"
#include "EVRP.hpp"
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

// This global solution object is declared in the restored heuristic.hpp
solution *best_sol;

// Helper function for the 2-Opt swap
static void two_opt_swap(int* tour, int size, int i, int j) {
    std::reverse(tour + i, tour + j);
}

/* Initializes the global best_sol structure */
void initialize_heuristic() {
    best_sol = new solution;
    best_sol->tour = new int[ACTUAL_PROBLEM_SIZE * 2]; // Allocate generous memory
    best_sol->steps = 0;
    best_sol->tour_length = DBL_MAX;
}

/* Implements the Simulated Annealing heuristic */
void run_heuristic() {
    // 1. Generate a valid initial solution (simple sequential build)
    int customer_count = 0;
    best_sol->tour[0] = DEPOT;
    best_sol->steps = 1;
    bool visited[NUM_OF_CUSTOMERS + 1] = {false};
    visited[0] = true; 

    while (customer_count < NUM_OF_CUSTOMERS) {
        bool customer_added_in_route = false;
        for (int i = 1; i <= NUM_OF_CUSTOMERS; i++) {
            if (!visited[i]) {
                best_sol->tour[best_sol->steps++] = i;
                visited[i] = true;
                customer_count++;
                customer_added_in_route = true;
            }
        }
        if (customer_added_in_route) {
            best_sol->tour[best_sol->steps++] = DEPOT;
        }
    }
    
    best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);

    if (best_sol->tour_length >= DBL_MAX) {
        cout << "Warning: Initial solution is infeasible." << endl;
        return; // Cannot run SA on an infeasible start
    }

    // 2. Setup for Simulated Annealing
    double T = 1000.0;
    double cooling_rate = 0.995;
    double min_temperature = 1.0;
    int iterations_per_temp = 1000;
    
    int* current_tour = new int[best_sol->steps];
    std::copy(best_sol->tour, best_sol->tour + best_sol->steps, current_tour);
    int current_tour_size = best_sol->steps;
    double current_energy = best_sol->tour_length;

    // 3. Main Simulated Annealing Loop
    while (T > min_temperature) {
        for (int i = 0; i < iterations_per_temp; ++i) {
            
            int* neighbor_tour = new int[current_tour_size];
            std::copy(current_tour, current_tour + current_tour_size, neighbor_tour);

            int idx1 = rand() % current_tour_size;
            int idx2 = rand() % current_tour_size;

            if (abs(idx1 - idx2) < 2) { 
                delete[] neighbor_tour;
                continue; 
            }
            if (idx1 > idx2) std::swap(idx1, idx2);
            
            // Avoid swapping depot at the start/end of a sub-route
            if (neighbor_tour[idx1] == DEPOT || neighbor_tour[idx2] == DEPOT) {
                delete[] neighbor_tour;
                continue;
            }

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
                best_sol->steps = current_tour_size;
                std::copy(current_tour, current_tour + current_tour_size, best_sol->tour);
            }
        }
        T *= cooling_rate;
    }

    delete[] current_tour;
}

/* Frees the memory allocated for the solution */
void free_heuristic() {
    delete[] best_sol->tour;
    delete best_sol;
}