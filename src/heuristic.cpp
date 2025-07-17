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
// Create a single, high-quality random number generator for the entire heuristic
static std::mt19937 g(std::chrono::high_resolution_clock::now().time_since_epoch().count());

// --- Helper Functions ---
static void two_opt_swap(int *tour, int size, int i, int j)
{
    std::reverse(tour + i, tour + j + 1);
}
// Takes a list of customers and partitions them into routes for multiple vehicles
static std::vector<int> partition_customers(const std::vector<int> &customers, std::mt19937 &generator)
{
    std::vector<int> partitioned_tour = customers;
    std::uniform_int_distribution<int> distribution(1, partitioned_tour.size());
    for (int i = 0; i < MIN_VEHICLES - 1; i++)
    {
        partitioned_tour.insert(partitioned_tour.begin() + distribution(generator), DEPOT);
    }
    return partitioned_tour;
}

const char *get_base_filename(const char *path)
{
    const char *slash = strrchr(path, '/');
    const char *backslash = strrchr(path, '\\');
    if (slash && backslash)
    {
        return (slash > backslash) ? slash + 1 : backslash + 1;
    }
    if (slash)
        return slash + 1;
    if (backslash)
        return backslash + 1;
    return path;
}
// Creates a valid, feasible initial tour to start the search
static void create_initial_solution()
{
    while (true)
    {
        best_sol->tour[0] = DEPOT;
        best_sol->steps = 1;
        std::vector<int> customers_to_visit;
        for (int i = 0; i < NUM_OF_CUSTOMERS; i++)
        {
            customers_to_visit.push_back(i + 2);
        }
        std::shuffle(customers_to_visit.begin(), customers_to_visit.end(), g); // Use the global generator

        std::vector<int> full_tour_path = partition_customers(customers_to_visit, g); // Pass the generator
        std::copy(full_tour_path.begin(), full_tour_path.end(), &best_sol->tour[best_sol->steps]);
        best_sol->steps += full_tour_path.size();
        best_sol->tour[best_sol->steps++] = DEPOT;

        best_sol->tour_length = get_solution_cost(best_sol->tour, best_sol->steps);

        // If the returned cost is less than DBL_MAX, the solution is valid.
        if (best_sol->tour_length < DBL_MAX)
        {
            // A feasible solution has been found, so we can exit the loop.
            break;
        }
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
static int find_best_charging_station(int from_node, int to_node, double energy_at_from)
{
    int best_cs = -1;
    double min_detour = DBL_MAX;

    // Find the best charging station to insert.
    for (int cs_id = NUM_OF_CUSTOMERS + 2; cs_id < ACTUAL_PROBLEM_SIZE; cs_id++)
    {
        // Check if we can reach the station from our current location
        if (get_energy_consumption(from_node, cs_id) <= (BATTERY_CAPACITY - energy_at_from))
        {
            // Check if we can reach the final destination AFTER charging at the station
            if (BATTERY_CAPACITY >= get_energy_consumption(cs_id, to_node))
            {
                double detour_dist = get_distance(from_node, cs_id) + get_distance(cs_id, to_node) - get_distance(from_node, to_node);
                if (detour_dist < min_detour)
                {
                    min_detour = detour_dist;
                    best_cs = cs_id;
                }
            }
        }
    }
    return best_cs;
}
static bool repair_tour(int *tour, int &size)
{
    double current_demand = 0.0;
    double current_energy = 0.0;

    for (int i = 0; i < size - 2; i++)
    {
        int from = tour[i];
        int to = tour[i + 1];
        // --- NEW: Proactive Capacity Check ---
        // Check if serving the NEXT node ('to') would violate capacity. This excludes depot and charging stations.
        if (to > 1 && to <= NUM_OF_CUSTOMERS) // to is a customer
        {
            if (current_demand + get_customer_demand(to) > MAX_CAPACITY)
            {
                // CAPACITY VIOLATION: We must insert a depot trip BEFORE visiting this customer.
                
                // Safety check: ensure there's space in the array to add a node.
                if (size >= (ACTUAL_PROBLEM_SIZE * 2) - 1)
                    return false; 

                // Shift the tour array to make space for the new depot visit at position i + 1.
                memmove(&tour[i + 2], &tour[i + 1], (size - (i + 1)) * sizeof(int));
                tour[i + 1] = DEPOT;
                size++;

                // The destination for this leg of the journey is now the depot.
                to = DEPOT; 
            }
        }

        // Check if the move to the next node is feasible
        if (current_energy + get_energy_consumption(from, to) > BATTERY_CAPACITY)
        {

            // Call the helper to find the best charging station
            int best_cs = find_best_charging_station(from, to, current_energy);

            if (best_cs != -1)
            {
                // A suitable station was found, so insert it into the tour.
                if (size >= (ACTUAL_PROBLEM_SIZE * 2) - 1)
                    return false; // Safety check

                memmove(&tour[i + 2], &tour[i + 1], (size - (i + 1)) * sizeof(int));
                tour[i + 1] = best_cs;
                size++;

                // After inserting, energy is reset as if we recharged.
                current_energy = 0.0;
            }
            else
            {
                // If no suitable charging station can fix the route, the repair fails.
                return false;
            }
        }

        // Update energy and demand for the current leg of the journey
        current_energy += get_energy_consumption(from, to);
        if (to == DEPOT)
        {
            current_demand = 0.0;
            current_energy = 0.0;
        }
        else
        {
            current_demand += get_customer_demand(to);
        }

        // The capacity check remains a hard constraint
        if (current_demand > MAX_CAPACITY)
        {
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
// Creates a new neighbor solution by performing a 2-Opt swap on the current tour.
// Creates a new, VALID neighbor solution by performing a 2-Opt swap.
static int *create_neighbor_solution(const int *tour, int tour_size, int &new_size, std::mt19937 &generator)
{
    if (tour_size <= 3)
    {
        return nullptr;
    }

    // Create a temporary neighbor to modify
    int *neighbor_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
    std::copy(tour, tour + tour_size, neighbor_tour);
    new_size = tour_size;

    // Generate indices and perform the swap
    std::uniform_int_distribution<int> distribution(1, new_size - 2);
    int idx1 = distribution(generator);
    int idx2 = distribution(generator);
    while (idx1 == idx2)
    {
        idx2 = distribution(generator);
    }
    if (idx1 > idx2)
        std::swap(idx1, idx2);
    two_opt_swap(neighbor_tour, new_size, idx1, idx2);

    // After swapping, immediately check if the new tour is valid.
    if (get_solution_cost(neighbor_tour, new_size) < DBL_MAX)
    {
        return neighbor_tour; // The swap resulted in a valid tour
    }
    else
    {
        delete[] neighbor_tour; // The swap resulted in an invalid tour, discard it
        return nullptr;
    }
}
// Attempts to merge two adjacent routes in the tour.
// Returns a new tour array if successful, otherwise returns nullptr.
static int *attempt_route_merge(const int *tour, int tour_size, int &new_size)
{
    std::vector<int> depot_indices;
    for (int i = 0; i < tour_size; ++i)
    {
        if (tour[i] == DEPOT)
        {
            depot_indices.push_back(i);
        }
    }

    // Cannot merge if there's only one route (or less)
    if (depot_indices.size() <= 2)
    {
        return nullptr;
    }

    // Pick a random depot to remove (but not the first or last one)
    int depot_to_remove_idx_in_vector = 1 + rand() % (depot_indices.size() - 2);
    int depot_to_remove_pos = depot_indices[depot_to_remove_idx_in_vector];

    // Create the new tour with one less node
    new_size = tour_size - 1;
    int *new_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
    int new_tour_idx = 0;
    for (int i = 0; i < tour_size; ++i)
    {
        if (i == depot_to_remove_pos)
        {
            continue; // Skip the depot to merge the routes
        }
        new_tour[new_tour_idx++] = tour[i];
    }

    // Check if the new, merged route is valid
    if (get_solution_cost(new_tour, new_size) < DBL_MAX)
    {
        return new_tour; // The merged route is valid
    }
    else
    {
        delete[] new_tour; // The merged route is invalid, so discard it
        return nullptr;
    }
}
void run_heuristic()
{
    for (int i = 0; i < ITERATIONS_PER_CALL; ++i)
    {

        int neighbor_tour_size = current_tour_size;
        int *neighbor_tour = nullptr; // null pointer

        // Randomly choose between a 2-Opt swap (more frequent) and a route merge
        if ((rand() % 10) < 8)
        { // 80% chance for a 2-Opt swap
            neighbor_tour = create_neighbor_solution(current_tour, current_tour_size, neighbor_tour_size, g);
        }
        else
        { // 20% chance for a route merge
            neighbor_tour = attempt_route_merge(current_tour, neighbor_tour_size, neighbor_tour_size);
        }

        if (neighbor_tour == nullptr)
        {
            continue; // Skip if the move was not possible or created an invalid merge
        }

        // Get the cost of the new solution (which includes repairing it if necessary)
        double neighbor_energy = get_solution_cost(neighbor_tour, neighbor_tour_size);

        // Decide whether to accept the new solution
        if (neighbor_energy < DBL_MAX)
        {
            if (neighbor_energy < current_energy)
            {
                // Always accept better solutions
                current_tour_size = neighbor_tour_size;
                std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
                current_energy = neighbor_energy;
            }
            else
            {
                // Probabilistically accept worse solutions
                double acceptance_prob = exp((current_energy - neighbor_energy) / T);
                if ((double)rand() / RAND_MAX < acceptance_prob)
                {
                    current_tour_size = neighbor_tour_size;
                    std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
                    current_energy = neighbor_energy;
                }
            }
        }

        delete[] neighbor_tour;

        // Update the overall best solution if the current one is better
        if (current_energy < best_sol->tour_length)
        {
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