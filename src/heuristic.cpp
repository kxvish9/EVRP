#include "heuristic.hpp"
#include "EVRP.hpp"
#include <iostream>  // For cout
#include <vector>    // For std::vector
#include <algorithm> // For std::shuffle
#include <random>    // For std::mt19937
#include <chrono>    // For std::chrono
#include <cfloat>    // For DBL_MAX
#include <cmath>     // For exp()
#include <string.h>
using namespace std;

// Global solution object, declared in heuristic.hpp
solution *best_sol;

// Global variables to maintain the state of the SA algorithm across multiple calls
static int *current_tour = nullptr;
static int current_tour_size = 0;
static double current_energy = DBL_MAX;
static double T = 1000.0; // Temperature
static const double COOLING_RATE = 0.99;
static const int ITERATIONS_PER_CALL = 100;

// --- Helper Functions ---
static void two_opt_swap(int *tour, int size, int i, int j)
{
    std::reverse(tour + i, tour + j + 1);
}
// Takes a list of customers and partitions them into routes for multiple vehicles
static std::vector<int> partition_customers(const std::vector<int>& customers) {
    std::vector<int> partitioned_tour = customers;

    // Create a high-quality random number generator
    unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 g(seed);

    // To create 'MIN_VEHICLES' number of routes, insert 'MIN_VEHICLES - 1' depots.
    for (int i = 0; i < MIN_VEHICLES - 1; i++) {
        // Define a uniform distribution for random positions
        std::uniform_int_distribution<int> distribution(1, partitioned_tour.size());
        // Insert a depot at a random position using the better generator
        partitioned_tour.insert(partitioned_tour.begin() + distribution(g), DEPOT);
    }
    return partitioned_tour;
}
// This function checks validity, tries to repair if needed, and returns the final cost.
const char* get_base_filename(const char* path) {
    const char* slash = strrchr(path, '/');
    const char* backslash = strrchr(path, '\\');
    if (slash && backslash) {
        return (slash > backslash) ? slash + 1 : backslash + 1;
    }
    if (slash) return slash + 1;
    if (backslash) return backslash + 1;
    return path;
}
// Creates a valid, feasible initial tour to start the search
static void create_initial_solution() {
    // 1. Create a shuffled list of all customers
    std::vector<int> customers_to_visit;
    for (int i = 0; i < NUM_OF_CUSTOMERS; i++) {
        customers_to_visit.push_back(i + 2);
    }
    unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 g(seed);
    std::shuffle(customers_to_visit.begin(), customers_to_visit.end(), g);

    // 2. Partition the customer list into a multi-vehicle tour using the helper
    std::vector<int> full_tour_path = partition_customers(customers_to_visit);

    // 3. Build the final solution object
    best_sol->tour[0] = DEPOT;
    best_sol->steps = 1;
    std::copy(full_tour_path.begin(), full_tour_path.end(), &best_sol->tour[best_sol->steps]);
    best_sol->steps += full_tour_path.size();
    best_sol->tour[best_sol->steps++] = DEPOT;

    // 4. Evaluate the new tour
    best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);

    if (best_sol->tour_length >= DBL_MAX) {
        cout << "CRITICAL WARNING: The initial multi-vehicle solution is infeasible." << endl;
    }
}
// --- Main Heuristic Functions ---

void initialize_heuristic()
{
    // This check ensures memory is allocated only ONCE for the entire program execution.
    if (best_sol == nullptr)
    {
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
// Finds the best charging station to insert between two nodes to fix an energy deficit.
// Returns the ID of the best station, or -1 if no suitable station is found.
static int find_best_charging_station(int from_node, int to_node, double energy_at_from) {
    int best_cs = -1;
    double min_detour = DBL_MAX;

    // Find the best charging station to insert.
    for (int cs_id = NUM_OF_CUSTOMERS + 2; cs_id < ACTUAL_PROBLEM_SIZE; cs_id++) {
        // Check if we can reach the station from our current location
        if (get_energy_consumption(from_node, cs_id) <= energy_at_from) {
            // Check if we can reach the final destination AFTER charging at the station
            if (BATTERY_CAPACITY >= get_energy_consumption(cs_id, to_node)) {
                double detour_dist = get_distance(from_node, cs_id) + get_distance(cs_id, to_node) - get_distance(from_node, to_node);
                if (detour_dist < min_detour) {
                    min_detour = detour_dist;
                    best_cs = cs_id;
                }
            }
        }
    }
    return best_cs;
}
static bool repair_tour(int* tour, int& size) {
    double current_demand = 0.0;
    double current_energy = 0.0;

    for (int i = 0; i < size - 2; i++) {
        int from = tour[i];
        int to = tour[i + 1];

        // Check if the move to the next node is feasible
        if (current_energy + get_energy_consumption(from, to) > BATTERY_CAPACITY) {
            
            // Call the helper to find the best charging station
            int best_cs = find_best_charging_station(from, to, current_energy);

            if (best_cs != -1) {
                // A suitable station was found, so insert it into the tour.
                if (size >= (ACTUAL_PROBLEM_SIZE * 2) - 1) return false; // Safety check
                
                memmove(&tour[i + 2], &tour[i + 1], (size - (i + 1)) * sizeof(int));
                tour[i + 1] = best_cs;
                size++;
                
                // After inserting, energy is reset as if we recharged.
                current_energy = 0.0;
            } else {
                // If no suitable charging station can fix the route, the repair fails.
                return false;
            }
        }
        
        // Update energy and demand for the current leg of the journey
        current_energy += get_energy_consumption(from, to);
        if (to == DEPOT) {
            current_demand = 0.0;
            current_energy = 0.0;
        } else {
            current_demand += get_customer_demand(to);
        }

        // The capacity check remains a hard constraint
        if (current_demand > MAX_CAPACITY) {
            return false; // Repair failed.
        }
    }

    return true; // The entire tour is now feasible.
}
static double get_solution_cost(int *tour, int &size)
{
    int original_size = size;
    int *temp_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
    std::copy(tour, tour + size, temp_tour);

    if (repair_tour(temp_tour, size))
    {
        // Repair was successful.
        double final_dist = fitness_evaluation(temp_tour, size);
        // Copy the repaired tour back to the original.
        std::copy(temp_tour, temp_tour + size, tour);
        delete[] temp_tour;
        return final_dist;
    }
    else
    {
        // Repair failed. The solution is invalid.
        size = original_size; // Restore original size.
        delete[] temp_tour;
        return DBL_MAX;
    }
}
// Creates a new neighbor solution by performing a 2-Opt swap on the current tour.
// Note: This function allocates new memory that must be deleted by the caller.
static int* create_neighbor_solution(const int* tour, int tour_size) {
    if (tour_size <= 3) {
        return nullptr; // Not enough nodes to perform a valid swap
    }

    int* neighbor_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
    std::copy(tour, tour + tour_size, neighbor_tour);

    // Generate two random, distinct indices for the 2-Opt swap
    int idx1 = 1 + rand() % (tour_size - 2);
    int idx2 = 1 + rand() % (tour_size - 2);

    // Ensure the indices are different
    while (idx1 == idx2) {
        idx2 = 1 + rand() % (tour_size - 2);
    }
    
    if (idx1 > idx2) std::swap(idx1, idx2);

    two_opt_swap(neighbor_tour, tour_size, idx1, idx2);

    return neighbor_tour;
}
void run_heuristic() {
    for (int i = 0; i < ITERATIONS_PER_CALL; ++i) {
        
        // 1. Create a new neighbor solution
        int neighbor_tour_size = current_tour_size;
        int* neighbor_tour = create_neighbor_solution(current_tour, neighbor_tour_size);
        if (neighbor_tour == nullptr) {
            continue; // Skip if tour is too small to modify
        }

        // 2. Get the cost of the new solution (which includes repairing it if necessary)
        double neighbor_energy = get_solution_cost(neighbor_tour, neighbor_tour_size);

        // 3. Decide whether to accept the new solution
        if (neighbor_energy < DBL_MAX) {
            if (neighbor_energy < current_energy) {
                // Always accept better solutions
                current_tour_size = neighbor_tour_size;
                std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
                current_energy = neighbor_energy;
            } else {
                // Probabilistically accept worse solutions
                double acceptance_prob = exp((current_energy - neighbor_energy) / T);
                if ((double)rand() / RAND_MAX < acceptance_prob) {
                    current_tour_size = neighbor_tour_size;
                    std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
                    current_energy = neighbor_energy;
                }
            }
        }
        
        // 4. Clean up memory for the neighbor
        delete[] neighbor_tour;

        // 5. Update the overall best solution if the current one is better
        if (current_energy < best_sol->tour_length) {
            best_sol->tour_length = current_energy;
            std::copy(current_tour, current_tour + current_tour_size, best_sol->tour);
            best_sol->steps = current_tour_size;
        }
    }

    // Cool the temperature after each batch of iterations
    T *= COOLING_RATE;
}

void free_heuristic()
{
    delete[] best_sol->tour;
    best_sol = nullptr;
    delete[] current_tour; // Free the memory for the current tour state
    current_tour = nullptr;
}