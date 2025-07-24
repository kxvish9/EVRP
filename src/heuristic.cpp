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
// ADDED: Reusable buffers to avoid memory allocation in loops
static int *neighbor_tour = nullptr;
static int *repair_buffer = nullptr;
static double *g_nearest_station_energy_cache = nullptr;
// ADDED: A proper uniform distribution for the acceptance check
static std::uniform_real_distribution<double> g_unif_dist(0.0, 1.0);
// --- Helper Functions ---
static void populate_nearest_station_cache()
{
    g_nearest_station_energy_cache = new double[ACTUAL_PROBLEM_SIZE];
    for (int i = 0; i < ACTUAL_PROBLEM_SIZE; ++i)
    {
        double min_energy = DBL_MAX;
        for (int cs_id = NUM_OF_CUSTOMERS + 1; cs_id < ACTUAL_PROBLEM_SIZE; ++cs_id)
        {
            double energy_cost = get_energy_consumption(i, cs_id);
            if (energy_cost < min_energy)
            {
                min_energy = energy_cost;
            }
        }
        g_nearest_station_energy_cache[i] = min_energy;
    }
}
static void two_opt_swap(int *tour, int size, int i, int j)
{
    std::reverse(tour + i, tour + j + 1);
}
// Takes a list of customers and partitions them into routes for multiple vehicles
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
// It takes any tour and makes it feasible by inserting depots and charging stations.
// It returns 'true' if a feasible tour could be constructed.
// REPLACEMENT: A more robust and clearer version of make_feasible
// Debug-enabled version of make_feasible
// Calculates the energy needed to travel from a node to the nearest charging station.
static double get_energy_to_reach_nearest_station(int from_node)
{
    return g_nearest_station_energy_cache[from_node];
}
static bool make_feasible(int *tour, int &size)
{
    std::vector<int> feasible_tour;
    feasible_tour.push_back(0); // Start at Depot

    double current_demand = 0.0;
    double current_energy = BATTERY_CAPACITY;

    // Iterate through the destinations of the original tour
    for (int i = 1; i < size; ++i)
    {
        int to_node = tour[i];
        int from_node = feasible_tour.back();
        double escape_energy = get_energy_to_reach_nearest_station(to_node);
        double energy_at_destination = current_energy - get_energy_consumption(from_node, to_node);

        if (energy_at_destination < escape_energy)
        {
            // If we won't have enough energy to escape from the destination, refuel now.
            int cs = find_best_charging_station(from_node, to_node, current_energy);
            if (cs != -1)
            {
                feasible_tour.push_back(cs);
                current_energy = BATTERY_CAPACITY;
                from_node = cs; // Update our current location
            }
            else
            {
                return false; // The risky situation is unfixable.
            }
        }
        // --- Step 1: Handle Capacity Failures First ---
        // Does adding the NEXT customer exceed capacity? If so, we must go to the depot NOW.
        if ((to_node > 0 && !is_charging_station(to_node)) && (current_demand + get_customer_demand(to_node) > MAX_CAPACITY))
        {

            // Check if we have enough energy to get to the depot
            if (current_energy < get_energy_consumption(from_node, 0))
            {
                // Not enough energy to reach the depot, so we must find a CS first.
                int cs = find_best_charging_station(from_node, 0, current_energy);
                if (cs != -1)
                {
                    feasible_tour.push_back(cs);
                    current_energy = BATTERY_CAPACITY; // Recharged at CS
                    from_node = cs;                    // Update our current location
                }
                else
                {
                    return false; // Can't find a CS to get to the depot, tour is unfixable
                }
            }
            feasible_tour.push_back(0);
            current_demand = 0.0;
            // IMPORTANT: Upon arrival at the depot, energy is fully restored for the NEXT trip.
            current_energy = BATTERY_CAPACITY;
            from_node = 0;
        }
        // --- Step 2: Handle Energy Failures for the main trip ---
        // Now, we calculate the trip from our current last location to the intended destination.

        if (current_energy < get_energy_consumption(from_node, to_node))
        {
            // Not enough energy, find a CS for the from_node -> to_node leg.
            int cs = find_best_charging_station(from_node, to_node, current_energy);
            if (cs != -1)
            {
                feasible_tour.push_back(cs);
                current_energy = BATTERY_CAPACITY; // Recharged at CS
                from_node = cs;                    // Update our current location
            }
            else
            {
                return false; // Can't find a CS for this leg, tour is unfixable
            }
        }

        // --- Step 3: Execute the trip and update state ---
        current_energy -= get_energy_consumption(from_node, to_node);
        feasible_tour.push_back(to_node);

        if (to_node > 0 && !is_charging_station(to_node))
        {
            current_demand += get_customer_demand(to_node);
        }
        // NOTE: No need for an 'else if' for the depot here, because if we arrive at a depot,
        // the energy and demand were already reset during the capacity check step.
    }

    // --- Final Step: Copy the valid tour back ---
    if (feasible_tour.size() >= (size_t)(ACTUAL_PROBLEM_SIZE * 2))
        return false;
    size = feasible_tour.size();
    std::copy(feasible_tour.begin(), feasible_tour.end(), tour);
    return true;
}
static void create_initial_solution()
{
    std::vector<int> unvisited_customers;
    for (int i = 0; i < NUM_OF_CUSTOMERS; ++i)
    {
        unvisited_customers.push_back(i + 1);
    }

    // CHANGED: Depot is 1, as defined in your data file
    best_sol->tour[0] = 0;
    best_sol->steps = 1;

    std::uniform_int_distribution<int> dist(0, unvisited_customers.size() - 1);
    int first_customer_idx = dist(g);
    int current_node = unvisited_customers[first_customer_idx];
    best_sol->tour[best_sol->steps++] = current_node;
    unvisited_customers[first_customer_idx] = unvisited_customers.back();
    unvisited_customers.pop_back();

    while (!unvisited_customers.empty())
    {
        double min_dist = DBL_MAX;
        int nearest_customer_idx = -1;

        for (size_t i = 0; i < unvisited_customers.size(); ++i)
        {
            double dist = get_distance(current_node, unvisited_customers[i]);
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_customer_idx = i;
            }
        }

        current_node = unvisited_customers[nearest_customer_idx];
        best_sol->tour[best_sol->steps++] = current_node;
        unvisited_customers[nearest_customer_idx] = unvisited_customers.back();
        unvisited_customers.pop_back();
    }

    best_sol->tour[best_sol->steps++] = 0;
}

void initialize_heuristic()
{
    if (best_sol == nullptr)
    {
        best_sol = new solution;
        best_sol->tour = new int[ACTUAL_PROBLEM_SIZE * 2];
        current_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
        neighbor_tour = new int[ACTUAL_PROBLEM_SIZE * 2];
        repair_buffer = new int[ACTUAL_PROBLEM_SIZE * 2];
        populate_nearest_station_cache();
    }

    // --- Loop to guarantee a feasible solution is found ---
    while (true)
    {
        // 1. Create a logical tour (a sequence of customers)
        create_initial_solution();

        // 2. Make it feasible and calculate its real-world cost.
        best_sol->tour_length = get_solution_cost(best_sol->tour, best_sol->steps);

        // 3. If successful, break the loop. Otherwise, it will retry automatically.
        if (best_sol->tour_length < DBL_MAX)
        {
            break;
        }
    }

    // Setup the main algorithm state with the valid solution
    current_tour_size = best_sol->steps;
    std::copy(best_sol->tour, best_sol->tour + current_tour_size, current_tour);
    current_energy = best_sol->tour_length;

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
    for (int cs_id = NUM_OF_CUSTOMERS + 1; cs_id < ACTUAL_PROBLEM_SIZE; cs_id++)
    {
        // Check if we can reach the station from our current location
        if (get_energy_consumption(from_node, cs_id) <= (energy_at_from))
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

// ADD THIS MISSING FUNCTION
static double get_solution_cost(int *tour, int &size)
{
    // Use the global repair_buffer to work on a copy.
    int original_size = size;
    std::copy(tour, tour + original_size, repair_buffer);

    // First, make the tour in the buffer feasible.
    if (make_feasible(repair_buffer, size))
    {
        // If successful, copy the repaired tour back to the original array.
        std::copy(repair_buffer, repair_buffer + size, tour);
        // Then, return the final cost of the valid tour.
        return fitness_evaluation(tour, size);
    }
    else
    {
        // If it can't be made feasible, restore the original size and return an invalid cost.
        size = original_size;
        return DBL_MAX;
    }
}
// Creates a new neighbor solution by performing a 2-Opt swap on the current tour.
// Note: This function allocates new memory that must be deleted by the caller.
// Creates a new neighbor solution by performing a 2-Opt swap on the current tour.
// Creates a new, VALID neighbor solution by performing a 2-Opt swap.
// CHANGED: This function now just performs the swap on the provided buffer.
// It doesn't allocate memory or check for validity.
static void create_neighbor_solution(int *neighbor_buffer, const int *tour, int tour_size, std::mt19937 &generator)
{
    if (tour_size <= 3)
    {
        // If we can't create a neighbor, just copy the original
        std::copy(tour, tour + tour_size, neighbor_buffer);
        return;
    }

    // Copy the original tour into the neighbor buffer to prepare for modification
    std::copy(tour, tour + tour_size, neighbor_buffer);

    // Generate indices and perform the swap
    std::uniform_int_distribution<int> distribution(1, tour_size - 2);
    int idx1 = distribution(generator);
    int idx2 = distribution(generator);
    while (idx1 == idx2)
    {
        idx2 = distribution(generator);
    }
    if (idx1 > idx2)
        std::swap(idx1, idx2);
    two_opt_swap(neighbor_buffer, tour_size, idx1, idx2);
}
// Attempts to merge two adjacent routes in the tour.
// Returns a new tour array if successful, otherwise returns nullptr.

void run_heuristic()
{
    for (int i = 0; i < ITERATIONS_PER_CALL; ++i)
    {

        int neighbor_tour_size = current_tour_size;

        create_neighbor_solution(neighbor_tour, current_tour, current_tour_size, g);

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
                if (g_unif_dist(g) < acceptance_prob)
                {
                    current_tour_size = neighbor_tour_size;
                    std::copy(neighbor_tour, neighbor_tour + neighbor_tour_size, current_tour);
                    current_energy = neighbor_energy;
                }
            }
        }

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
    delete best_sol;
    best_sol = nullptr;
    delete[] current_tour;
    current_tour = nullptr;

    // ADDED: Free the new buffers
    delete[] neighbor_tour;
    neighbor_tour = nullptr;
    delete[] repair_buffer;
    repair_buffer = nullptr;
    if (g_nearest_station_energy_cache != nullptr)
    {
        delete[] g_nearest_station_energy_cache;
        g_nearest_station_energy_cache = nullptr;
    }
}