#include "heuristic.hpp"
#include "EVRP.hpp"
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>      // For std::mt19937
#include <chrono>
#include <stdio.h>    
#include <vector> // For seeding the random number generator
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
// This function checks validity, tries to repair if needed, and returns the final cost.

// Creates a valid, feasible initial tour to start the search
static void create_initial_solution() {
    // Start at the depot
    best_sol->tour[0] = DEPOT;
    best_sol->steps = 1;

    // Create a list of all customers to be visited
    std::vector<int> customers_to_visit;
    for (int i = 0; i < NUM_OF_CUSTOMERS; i++) {
        customers_to_visit.push_back(i + 2);
    }

    // Shuffle the customer list to randomize the order
    unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 g(seed);
    std::shuffle(customers_to_visit.begin(), customers_to_visit.end(), g);

    // --- Start of New Logic ---

    // To create 'VEHICLES' number of routes, we need to insert 'VEHICLES - 1' depots.
    // This partitions the shuffled customer list into separate routes.
    for (int i = 0; i < MIN_VEHICLES - 1; i++) {
        // Insert a depot at a random position within the customer list
        // We avoid position 0 to prevent empty routes like 1-1.
        int pos = 1 + rand() % customers_to_visit.size();
        customers_to_visit.insert(customers_to_visit.begin() + pos, DEPOT);
    }

    // --- End of New Logic ---

    // Append the now-partitioned list of customers and depots to the tour
    for (int node_id : customers_to_visit) {
        best_sol->tour[best_sol->steps++] = node_id;
    }
    
    // End the entire tour at the depot
    best_sol->tour[best_sol->steps++] = DEPOT;
    
    // Evaluate the complete multi-vehicle tour
    best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);

    if (best_sol->tour_length >= DBL_MAX) {
        cout << "CRITICAL WARNING: The initial multi-vehicle solution is infeasible." << endl;
    }
}
// --- Main Heuristic Functions ---

void initialize_heuristic() {
    // This check ensures memory is allocated only ONCE for the entire program execution.
    if (best_sol == nullptr) {
        best_sol = new solution;
        best_sol->tour = new int[ACTUAL_PROBLEM_SIZE * 2];
        current_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
    }

    // This part will now run at the beginning of every trial, resetting the state.

    create_initial_solution();
    
    current_tour_size = best_sol->steps;
    std::copy(best_sol->tour, best_sol->tour + current_tour_size, current_tour);
    current_energy = best_sol->tour_length;
    
    // Reset temperature for the start of a new run
    T = 1000.0;
    
}
// This new function attempts to repair an infeasible tour by inserting a charging station.
// It returns 'true' if the repair was successful, and 'false' otherwise.
static bool repair_tour(int* tour, int& size) {
    double current_demand = 0.0;
    double current_energy = 0.0;

    for (int i = 0; i < size - 2; i++) {
        int from = tour[i];
        int to = tour[i + 1];

        // Check if the move to the next node is feasible
        if (current_energy + get_energy_consumption(from, to) > BATTERY_CAPACITY) {
            // Infeasibility detected! We need to insert a charging station before node 'to'.
            int best_cs = -1;
            double min_detour = DBL_MAX;

            // Find the best charging station to insert.
            for (int cs_id = NUM_OF_CUSTOMERS + 2; cs_id < ACTUAL_PROBLEM_SIZE; cs_id++) {
                if (get_energy_consumption(from, cs_id) <= current_energy) { // Can we reach the station?
                    double detour_dist = get_distance(from, cs_id) + get_distance(cs_id, to) - get_distance(from, to);
                    if (detour_dist < min_detour) {
                        min_detour = detour_dist;
                        best_cs = cs_id;
                    }
                }
            }

            if (best_cs != -1) {
                // We found a charging station to insert.
                // Make space for the new node in the tour array.
                // Use memmove to safely shift the block of memory one position to the right
                memmove(&tour[i + 2], &tour[i + 1], (size - (i + 1)) * sizeof(int));
                // Insert the station and update the tour size.
                tour[i + 1] = best_cs;
                size++;
                
                // After inserting, we reset energy as if we recharged.
                current_energy = 0.0;
                // We also need to re-evaluate from the newly inserted station.
                // The 'i' counter in the main loop will now process the move from 'best_cs' to 'to'.
            } else {
                // If no suitable charging station can be found to fix the route.
                return false; // Repair failed.
            }
        }
        
        // Update energy and demand as before
        current_energy += get_energy_consumption(from, to);
        if (to == DEPOT) {
            current_demand = 0.0;
            current_energy = 0.0;
        } else {
            current_demand += get_customer_demand(to);
        }

        // The capacity check remains as a hard constraint. We can't easily "repair" it.
        if (current_demand > MAX_CAPACITY) {
            return false; // Repair failed.
        }
    }

    return true; // The entire tour is now feasible.
}
static double get_solution_cost(int* tour, int& size) {
    int original_size = size;
    int* temp_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
    std::copy(tour, tour + size, temp_tour);

    if (repair_tour(temp_tour, size)) {
        // Repair was successful.
        double final_dist = fitness_evaluation(temp_tour, size);
        // Copy the repaired tour back to the original.
        std::copy(temp_tour, temp_tour + size, tour);
        delete[] temp_tour;
        return final_dist;
    } else {
        // Repair failed. The solution is invalid.
        size = original_size; // Restore original size.
        delete[] temp_tour;
        return DBL_MAX; // <-- This return statement was missing
    }
}
void run_heuristic() {
    // This function now performs a small batch of SA iterations each time it's called.
    
    double cooling_rate = 0.9995;
    int iterations_per_call = 100; // Perform 100 swaps per call to run_heuristic

    for (int i = 0; i < iterations_per_call; ++i) {
        int* neighbor_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
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

        // This is the new size of the neighbor tour after potential repairs.
        int neighbor_tour_size = current_tour_size; 
        double neighbor_energy = get_solution_cost(neighbor_tour, neighbor_tour_size);

        if (neighbor_energy < DBL_MAX) {
            if (neighbor_energy < current_energy) {
                current_tour_size = neighbor_tour_size;
                std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
                current_energy = neighbor_energy;
            } else {
                double acceptance_prob = exp((current_energy - neighbor_energy) / T);
                if ((double)rand() / RAND_MAX < acceptance_prob) {
                    current_tour_size = neighbor_tour_size;
                    std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
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
int get_current_tour_size_debug() {
    return current_tour_size;
}