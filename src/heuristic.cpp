#include "heuristic.hpp"
#include "EVRP.hpp"
#include "stats.hpp"
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
static double initial_temperature;                  // To remember the starting temperature
static int iterations_without_improvement = 0;      // Counter for how long we've been stuck
static const int REHEAT_ITERATION_THRESHOLD = 5000; // How many iterations to wait before reheating
// ADDED: Reusable buffers to avoid memory allocation in loops
static int *neighbor_tour = nullptr;
static int *repair_buffer = nullptr;
static double *g_nearest_station_energy_cache = nullptr;
// ADDED: A proper uniform distribution for the acceptance check
static std::uniform_real_distribution<double> g_unif_dist(0.0, 1.0);
// heuristic.cpp

// ... (keep ALL your existing code, including run_heuristic(), initialize_heuristic(), etc.) ...


// =================================================================================
// --- NEW SECTION: GENETIC ALGORITHM IMPLEMENTATION ---
// =================================================================================

// --- GA Parameters ---
const int POPULATION_SIZE = 50;
const double MUTATION_RATE = 0.1;
const double CROSSOVER_RATE = 0.8;
const int TOURNAMENT_SIZE = 3;

// --- GA Data Structures ---
static solution population[POPULATION_SIZE];
static solution offspring[POPULATION_SIZE];

// --- GA Helper Functions ---

// Creates a random sequence of customers
static void create_customer_permutation(int* tour, int& size) {
    std::vector<int> customers;
    for (int i = 1; i <= NUM_OF_CUSTOMERS; ++i) {
        customers.push_back(i);
    }
    std::shuffle(customers.begin(), customers.end(), g);
    
    tour[0] = 0; // Start at depot
    size = 1;
    for (int customer : customers) {
        tour[size++] = customer;
    }
    tour[size++] = 0; // End at depot
}

// Selects a parent from the population using tournament selection
static const solution& tournament_selection() {
    int best_idx = -1;
    double best_fitness = DBL_MAX;
    std::uniform_int_distribution<int> dist(0, POPULATION_SIZE - 1);

    for (int i = 0; i < TOURNAMENT_SIZE; ++i) {
        int idx = dist(g);
        if (population[idx].tour_length < best_fitness) {
            best_fitness = population[idx].tour_length;
            best_idx = idx;
        }
    }
    return population[best_idx];
}

// Performs Ordered Crossover (OX1) on customer permutations
static void ordered_crossover(const solution& p1, const solution& p2, solution& child) {
    std::vector<int> p1_cust, p2_cust;
    for(int i = 0; i < p1.steps; ++i) if(p1.tour[i] > 0 && !is_charging_station(p1.tour[i])) p1_cust.push_back(p1.tour[i]);
    
    // If p1 had no customers, just copy p2
    if(p1_cust.empty()){
        child.steps = p2.steps;
        std::copy(p2.tour, p2.tour + p2.steps, child.tour);
        return;
    }

    for(int i = 0; i < p2.steps; ++i) if(p2.tour[i] > 0 && !is_charging_station(p2.tour[i])) p2_cust.push_back(p2.tour[i]);

    int size = p1_cust.size();
    std::uniform_int_distribution<int> dist(0, size - 1);
    int start = dist(g);
    int end = dist(g);
    if (start > end) std::swap(start, end);

    std::vector<int> child_cust(size, -1);
    std::vector<bool> in_child(ACTUAL_PROBLEM_SIZE, false);

    for (int i = start; i <= end; ++i) {
        child_cust[i] = p1_cust[i];
        in_child[p1_cust[i]] = true;
    }
    
    int p2_idx = 0;
    for (int i = 0; i < size; ++i) {
        if (child_cust[i] == -1) {
            while (p2_idx < p2_cust.size() && in_child[p2_cust[p2_idx]]) {
                p2_idx++;
            }
            if(p2_idx < p2_cust.size()){
                child_cust[i] = p2_cust[p2_idx];
                p2_idx++;
            }
        }
    }
    
    child.tour[0] = 0;
    child.steps = 1;
    for(int cust : child_cust) if(cust != -1) child.tour[child.steps++] = cust;
    child.tour[child.steps++] = 0;
}

// Applies a simple swap mutation
static void mutate(solution& sol) {
    swap_operator(sol.tour, sol.tour, sol.steps, g);
}


// --- Main GA Functions ---

void initialize_population() {
    // Note: The main initialize_heuristic already allocates best_sol.
    // We just need to allocate the population arrays.
    static bool ga_initialized = false;
    if (!ga_initialized) {
        for (int i = 0; i < POPULATION_SIZE; ++i) {
            population[i].tour = new int[ACTUAL_PROBLEM_SIZE * 2];
            offspring[i].tour = new int[ACTUAL_PROBLEM_SIZE * 2];
        }
        ga_initialized = true;
    }

    for (int i = 0; i < POPULATION_SIZE; ++i) {
        while (true) {
            create_customer_permutation(population[i].tour, population[i].steps);
            population[i].tour_length = get_solution_cost(population[i].tour, population[i].steps);
            if (population[i].tour_length < DBL_MAX) break;
        }
    }
    
    // Ensure best_sol is updated with the initial best
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        if (population[i].tour_length < best_sol->tour_length) {
            best_sol->tour_length = population[i].tour_length;
            best_sol->steps = population[i].steps;
            std::copy(population[i].tour, population[i].tour + population[i].steps, best_sol->tour);
        }
    }
}

void run_ga_generation() {
    // Elitism: copy the best individual
    int best_idx = 0;
    for (int i = 1; i < POPULATION_SIZE; ++i) {
        if (population[i].tour_length < population[best_idx].tour_length) best_idx = i;
    }
    std::copy(population[best_idx].tour, population[best_idx].tour + population[best_idx].steps, offspring[0].tour);
    offspring[0].steps = population[best_idx].steps;
    offspring[0].tour_length = population[best_idx].tour_length;

    // Create the rest of the offspring
    for (int i = 1; i < POPULATION_SIZE; ++i) {
        const solution& p1 = tournament_selection();
        const solution& p2 = tournament_selection();
        
        solution child;
        child.tour = offspring[i].tour; 

        if (g_unif_dist(g) < CROSSOVER_RATE) {
            ordered_crossover(p1, p2, child);
        } else {
            child.steps = p1.steps;
            std::copy(p1.tour, p1.tour + p1.steps, child.tour);
        }

        if (g_unif_dist(g) < MUTATION_RATE) {
            mutate(child);
        }

        double child_fitness = get_solution_cost(child.tour, child.steps);
        if(child_fitness >= DBL_MAX) {
            // If child is infeasible after repair, replace it with its first parent
            offspring[i].steps = p1.steps;
            offspring[i].tour_length = p1.tour_length;
            std::copy(p1.tour, p1.tour + p1.steps, offspring[i].tour);
        } else {
             offspring[i].steps = child.steps;
             offspring[i].tour_length = child_fitness;
        }
    }

    // Replace old population
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        population[i].steps = offspring[i].steps;
        population[i].tour_length = offspring[i].tour_length;
        std::copy(offspring[i].tour, offspring[i].tour + offspring[i].steps, population[i].tour);
    }

    // Update overall best solution
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        if (population[i].tour_length < best_sol->tour_length) {
            best_sol->tour_length = population[i].tour_length;
            best_sol->steps = population[i].steps;
            std::copy(population[i].tour, population[i].tour + population[i].steps, best_sol->tour);
        }
    }
    record_fitness(get_evals(), best_sol->tour_length);
}
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
#define OPERATOR_CHOICE 2

// OPERATOR 1: The original 2-Opt Swap
static void two_opt_operator(int *neighbor_buffer, const int *tour, int tour_size, std::mt19937 &generator)
{
    std::copy(tour, tour + tour_size, neighbor_buffer);
    if (tour_size <= 3)
        return;

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

// OPERATOR 2: A simple Customer Swap
static void swap_operator(int *neighbor_buffer, const int *tour, int tour_size, std::mt19937 &generator)
{
    std::copy(tour, tour + tour_size, neighbor_buffer);
    if (tour_size <= 3)
        return;

    std::uniform_int_distribution<int> dist(1, tour_size - 2);
    int idx1, idx2;
    int attempts = 0;
    const int max_attempts = 100;

    do
    {
        idx1 = dist(generator);
        idx2 = dist(generator);
        attempts++;
    } while (attempts < max_attempts && (idx1 == idx2 || is_charging_station(neighbor_buffer[idx1]) || neighbor_buffer[idx1] == DEPOT || is_charging_station(neighbor_buffer[idx2]) || neighbor_buffer[idx2] == DEPOT));

    if (attempts < max_attempts)
    {
        std::swap(neighbor_buffer[idx1], neighbor_buffer[idx2]);
    }
}

// OPERATOR 3: Ruin and Recreate (Large Neighborhood Search)
static void ruin_and_recreate_operator(int *neighbor_buffer, const int *tour, int tour_size, std::mt19937 &generator)
{
    std::vector<int> temp_tour(tour, tour + tour_size);

    int num_customers = 0;
    for (int node : temp_tour)
        if (node > 0 && !is_charging_station(node))
            num_customers++;

    if (num_customers < 2)
    {
        std::copy(tour, tour + tour_size, neighbor_buffer);
        return;
    }
    int num_to_remove = std::max(2, static_cast<int>(num_customers * 0.20));

    std::vector<int> customer_indices;
    for (int i = 1; i < temp_tour.size() - 1; ++i)
    {
        if (temp_tour[i] > 0 && !is_charging_station(temp_tour[i]))
        {
            customer_indices.push_back(i);
        }
    }
    std::shuffle(customer_indices.begin(), customer_indices.end(), generator);

    std::vector<int> removed_customers;
    std::vector<int> indices_to_remove;
    for (int i = 0; i < std::min((int)customer_indices.size(), num_to_remove); ++i)
    {
        indices_to_remove.push_back(customer_indices[i]);
    }
    std::sort(indices_to_remove.rbegin(), indices_to_remove.rend());

    for (int idx : indices_to_remove)
    {
        removed_customers.push_back(temp_tour[idx]);
        temp_tour.erase(temp_tour.begin() + idx);
    }

    for (int customer_to_insert : removed_customers)
    {
        double min_cost_increase = DBL_MAX;
        int best_insertion_pos = -1;

        for (int i = 0; i < temp_tour.size() - 1; ++i)
        {
            double cost_increase = get_distance(temp_tour[i], customer_to_insert) + get_distance(customer_to_insert, temp_tour[i + 1]) - get_distance(temp_tour[i], temp_tour[i + 1]);
            if (cost_increase < min_cost_increase)
            {
                min_cost_increase = cost_increase;
                best_insertion_pos = i + 1;
            }
        }

        if (best_insertion_pos != -1)
        {
            temp_tour.insert(temp_tour.begin() + best_insertion_pos, customer_to_insert);
        }
        else
        {
            temp_tour.insert(temp_tour.end() - 1, customer_to_insert);
        }
    }
    std::copy(temp_tour.begin(), temp_tour.end(), neighbor_buffer);
}

// --------------------------------------------------------------------------
static double get_energy_to_reach_nearest_station(int from_node)
{
    return g_nearest_station_energy_cache[from_node];
}
static bool make_feasible(int *tour, int &size)
{
    std::vector<int> feasible_tour;
    feasible_tour.push_back(0); // Start at Depot

    double current_demand = 0.0;
    double remaining_battery = BATTERY_CAPACITY;

    // Iterate through the destinations of the original tour
    for (int i = 1; i < size; ++i)
    {
        int to_node = tour[i];
        int from_node = feasible_tour.back();
        double escape_energy = get_energy_to_reach_nearest_station(to_node);
        double energy_at_destination = remaining_battery - get_energy_consumption(from_node, to_node);

        if (energy_at_destination < escape_energy)
        {
            // If we won't have enough energy to escape from the destination, refuel now.
            int cs = find_best_charging_station(from_node, to_node, remaining_battery);
            if (cs != -1)
            {
                feasible_tour.push_back(cs);
                remaining_battery = BATTERY_CAPACITY;
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
            if (remaining_battery < get_energy_consumption(from_node, 0))
            {
                // Not enough energy to reach the depot, so we must find a CS first.
                int cs = find_best_charging_station(from_node, 0, remaining_battery);
                if (cs != -1)
                {
                    feasible_tour.push_back(cs);
                    remaining_battery = BATTERY_CAPACITY; // Recharged at CS
                    from_node = cs;                       // Update our current location
                }
                else
                {
                    return false; // Can't find a CS to get to the depot, tour is unfixable
                }
            }
            feasible_tour.push_back(0);
            current_demand = 0.0;
            // IMPORTANT: Upon arrival at the depot, energy is fully restored for the NEXT trip.
            remaining_battery = BATTERY_CAPACITY;
            from_node = 0;
        }
        // --- Step 2: Handle Energy Failures for the main trip ---
        // Now, we calculate the trip from our current last location to the intended destination.

        if (remaining_battery < get_energy_consumption(from_node, to_node))
        {
            // Not enough energy, find a CS for the from_node -> to_node leg.
            int cs = find_best_charging_station(from_node, to_node, remaining_battery);
            if (cs != -1)
            {
                feasible_tour.push_back(cs);
                remaining_battery = BATTERY_CAPACITY; // Recharged at CS
                from_node = cs;                       // Update our current location
            }
            else
            {
                return false; // Can't find a CS for this leg, tour is unfixable
            }
        }

        // --- Step 3: Execute the trip and update state ---
        remaining_battery -= get_energy_consumption(from_node, to_node);
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

    initial_temperature = 1000.0; // Or calculate based on initial solution cost
    T = initial_temperature;
    iterations_without_improvement = 0;
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
// heuristic.cpp

// REPLACE the old create_neighbor_solution function with this controller.
static void create_neighbor_solution(int *neighbor_buffer, const int *tour, int tour_size, std::mt19937 &generator)
{
#if OPERATOR_CHOICE == 0
    // Use the original 2-Opt operator
    two_opt_operator(neighbor_buffer, tour, tour_size, generator);
#elif OPERATOR_CHOICE == 1
    // Use the simple customer swap operator
    swap_operator(neighbor_buffer, tour, tour_size, generator);
#elif OPERATOR_CHOICE == 2
    // Use the powerful Ruin & Recreate operator
    ruin_and_recreate_operator(neighbor_buffer, tour, tour_size, generator);
#endif
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
            record_fitness(get_evals(), neighbor_energy);
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
            iterations_without_improvement = 0;
        }
        else
        {
            // ADD THIS: If no improvement, increment the counter
            iterations_without_improvement++;
        }
    }

    // Cool the temperature after each batch of iterations
    T *= COOLING_RATE;
    if (iterations_without_improvement >= REHEAT_ITERATION_THRESHOLD)
    {
        T = initial_temperature * 0.6;      // Reheat to a high temperature
        iterations_without_improvement = 0; // Reset the counter
    }
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