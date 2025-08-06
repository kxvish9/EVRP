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
#include "config.hpp"
using namespace std;

// Global solution object, declared in heuristic.hpp
solution *best_sol;
// Global variables to maintain the state of the SA algorithm across multiple calls
static int *current_tour = nullptr;
static int current_tour_size = 0;
static double current_energy;
static double T;            // Temperature, loaded from config
static double COOLING_RATE; // Loaded from config
static int ITERATIONS_PER_CALL;
// Create a single, high-quality random number generator for the entire heuristic
static std::mt19937 g(std::chrono::high_resolution_clock::now().time_since_epoch().count());
static double initial_temperature;         // To remember the starting temperature
static int iterations_without_improvement; // Counter for how long we've been stuck
static long REHEAT_ITERATION_THRESHOLD;     // Loaded from config
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
        double min_energy = get_energy_consumption(i, DEPOT);
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

// OPERATOR 2: A simple Customer Swap
// OPERATOR 2: A simple Customer Swap (Robust Version)
static void swap_operator(int *neighbor_buffer, const int *tour, int &size, std::mt19937 &generator)
{
    std::copy(tour, tour + size, neighbor_buffer);

    // Step 1: Create a list of indices for all swappable nodes (i.e., customers).
    std::vector<int> customer_indices;
    for (int i = 1; i < size - 1; ++i)
    {
        // The loop bounds already exclude start/end depots. Now, also exclude stations.
        if (!is_charging_station(tour[i]))
        {
            customer_indices.push_back(i);
        }
    }

    // Safeguard: If there are fewer than two customers, a swap is impossible.
    if (customer_indices.size() < 2)
    {
        return; // Do nothing.
    }

    // Step 2: Shuffle the list of valid customer indices.
    std::shuffle(customer_indices.begin(), customer_indices.end(), generator);

    // Step 3: Pick the first two distinct indices from the shuffled list.
    int idx1 = customer_indices[0];
    int idx2 = customer_indices[1];

    // Step 4: Perform the swap.
    std::swap(neighbor_buffer[idx1], neighbor_buffer[idx2]);
}
static void remove_station_operator(int *neighbor_buffer, const int *tour, int &size, std::mt19937 &generator)
{
    std::vector<int> station_indices;
    // Find all charging stations in the current tour (excluding start/end depots)
    for (int i = 1; i < size - 1; ++i)
    {
        if (is_charging_station(tour[i]))
        {
            station_indices.push_back(i);
        }
    }

    // If there are no stations to remove, do nothing.
    if (station_indices.empty())
    {
        std::copy(tour, tour + size, neighbor_buffer);
        return;
    }

    // Pick a random station to remove
    std::uniform_int_distribution<int> dist(0, station_indices.size() - 1);
    int index_to_remove = station_indices[dist(generator)];

    // Use std::vector for easy removal
    std::vector<int> temp_tour(tour, tour + size);
    temp_tour.erase(temp_tour.begin() + index_to_remove);

    // Copy the result to the output buffer
    // The size is not passed by reference here, but get_solution_cost will handle the size change.
    size = temp_tour.size();
    std::copy(temp_tour.begin(), temp_tour.end(), neighbor_buffer);
}
// OPERATOR 3: Ruin and Recreate (Large Neighborhood Search)
static void ruin_and_recreate_operator(int *neighbor_buffer, const int *tour, int &size, std::mt19937 &generator)
{
    std::vector<int> temp_tour(tour, tour + size);

    int num_customers = 0;
    for (int node : temp_tour)
        if (!is_charging_station(node))
            num_customers++;

    if (num_customers < 2)
    {
        std::copy(tour, tour + size, neighbor_buffer);
        return;
    }
    int num_to_remove = std::max(2, static_cast<int>(num_customers * 0.20));

    std::vector<int> customer_indices;
    for (int i = 1; i < temp_tour.size() - 1; ++i)
    {
        if (!is_charging_station(temp_tour[i]))
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
    for (size_t i = 0; i + 1 < temp_tour.size();)
    {
        if (temp_tour[i] == temp_tour[i + 1] && is_charging_station(temp_tour[i]))
        {
            temp_tour.erase(temp_tour.begin() + i);
            // After erasing, don't increment i because we need to re-check
            // the new element at the current position.
        }
        else
        {
            i++; // Only move to the next position if no duplicate was found.
        }
    }
    size = temp_tour.size();
    std::copy(temp_tour.begin(), temp_tour.end(), neighbor_buffer);
}

static double get_energy_to_reach_nearest_station(int from_node)
{
    return g_nearest_station_energy_cache[from_node];
}
static bool make_feasible(int *tour, int &size)
{
    // --- PHASE 1: CAPACITY REPAIR ---
    // First, build a new tour that inserts depot visits wherever cargo
    // capacity would be exceeded. This pass preserves any existing charging
    // stations from the input tour but does not consider energy.
    std::vector<int> capacity_tour;
    capacity_tour.push_back(DEPOT);
    double current_demand = 0.0;

    for (int i = 1; i < size; ++i)
    {
        int current_node = tour[i];
        if (current_node == DEPOT)
        {
            continue; // Ignore depots in the tour, we'll add them as needed.
        }

        if (is_charging_station(current_node))
        {
            capacity_tour.push_back(current_node); // Preserve existing charging stations.
        }
        else
        { // Node is a customer
            if (current_demand + get_customer_demand(current_node) > MAX_CAPACITY)
            {
                capacity_tour.push_back(DEPOT); // Insert a depot visit before the customer.
                current_demand = 0.0;           // Reset demand.
            }
            capacity_tour.push_back(current_node); // Add the customer.
            current_demand += get_customer_demand(current_node);
        }
    }
    capacity_tour.push_back(DEPOT); // Ensure the tour always ends at the depot.

    // --- PHASE 2: ENERGY REPAIR ---
    // With a capacity-feasible tour, this pass inserts charging stations to
    // ensure the battery never runs out. It uses a forward-looking check to
    // prevent getting stranded.
    std::vector<int> final_tour;
    final_tour.push_back(DEPOT);
    double remaining_battery = BATTERY_CAPACITY;

    for (size_t i = 1; i < capacity_tour.size(); ++i)
    {
        int from_node = final_tour.back();
        int to_node = capacity_tour[i];

        // Calculate the energy state *after* the next trip.
        double energy_to_reach_dest = get_energy_consumption(from_node, to_node);
        double energy_at_dest = remaining_battery - energy_to_reach_dest;
        if (is_charging_station(to_node) && energy_at_dest > 0)
        {
            // If we reach a charging station, we can reset the battery.
            energy_at_dest = BATTERY_CAPACITY;
        }
        double escape_energy = get_energy_to_reach_nearest_station(to_node);

        // Do we need to refuel to complete the trip AND escape the destination?
        if (energy_at_dest < escape_energy)
        {
            // Find the best charging station to visit *before* 'to_node'.
            int best_cs = find_best_charging_station(from_node, to_node, remaining_battery);
            if (best_cs != -1)
            {
                // Detour to the charging station.
                if (best_cs == to_node || best_cs == from_node)
                {
                    return false; // Unfixable: No detour needed, but energy is insufficient.
                }
                final_tour.push_back(best_cs);
                remaining_battery = BATTERY_CAPACITY; // Battery is full upon arrival.
                from_node = best_cs;                  // Our new location is the charging station.
            }
            else
            {
                return false; // Unfixable: No suitable charging station found.
            }
        }

        // Now, execute the trip from our current location ('from_node') to the destination ('to_node').
        double energy_for_final_leg = get_energy_consumption(from_node, to_node);

        // This is a safeguard; it should not fail if find_best_charging_station is correct.
        if (remaining_battery < energy_for_final_leg)
        {
            return false;
        }

        remaining_battery -= energy_for_final_leg;
        final_tour.push_back(to_node);
        // If the destination is a refueling stop, reset the battery for the next leg.
        if (is_charging_station(to_node))
        {
            remaining_battery = BATTERY_CAPACITY;
        }
    }

    // --- FINALIZATION: Copy the valid tour back ---
    if (final_tour.size() >= (size_t)(ACTUAL_PROBLEM_SIZE * 2))
    {
        return false; // Tour is excessively long, likely an error state.
    }

    size = final_tour.size();
    std::copy(final_tour.begin(), final_tour.end(), tour);
    return true;
}
static void create_initial_solution()
{
    std::vector<int> unvisited_customers;
    for (int i = 1; i <= NUM_OF_CUSTOMERS; ++i)
    {
        unvisited_customers.push_back(i);
    }
    // Start with a random customer to break ties
    std::shuffle(unvisited_customers.begin(), unvisited_customers.end(), g);

    // 1. Create the initial tour with one customer: DEPOT -> C1 -> DEPOT
    std::vector<int> tour;
    tour.push_back(DEPOT);
    tour.push_back(unvisited_customers.back());
    unvisited_customers.pop_back();
    tour.push_back(DEPOT);

    // 2. Iteratively insert the remaining customers
    while (!unvisited_customers.empty())
    {
        double min_cost = DBL_MAX;
        int best_customer_idx = -1;
        int best_insertion_pos = -1;

        // For every unvisited customer...
        for (int i = 0; i < unvisited_customers.size(); ++i)
        {
            int customer_to_insert = unvisited_customers[i];

            // ...find the cheapest place to insert it in the current tour
            for (int j = 0; j < tour.size() - 1; ++j)
            {
                int node_before = tour[j];
                int node_after = tour[j + 1];

                // Calculate the cost of inserting the customer between these two nodes
                double insertion_cost = get_distance(node_before, customer_to_insert) + get_distance(customer_to_insert, node_after) - get_distance(node_before, node_after);

                if (insertion_cost < min_cost)
                {
                    min_cost = insertion_cost;
                    best_customer_idx = i;
                    best_insertion_pos = j + 1;
                }
            }
        }

        // 3. Perform the best insertion found in this iteration
        if (best_customer_idx != -1)
        {
            tour.insert(tour.begin() + best_insertion_pos, unvisited_customers[best_customer_idx]);
            unvisited_customers.erase(unvisited_customers.begin() + best_customer_idx);
        }
        else
        {
            // Failsafe in case no insertion is found (should not happen)
            break;
        }
    }

    // 4. Copy the final tour to the solution object
    best_sol->steps = tour.size();
    std::copy(tour.begin(), tour.end(), best_sol->tour);
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
    record_fitness(get_evals(), best_sol->tour_length);
    initial_temperature = g_config.sa_initial_temp;
    T = initial_temperature;
    COOLING_RATE = g_config.sa_cooling_rate;
    REHEAT_ITERATION_THRESHOLD = g_config.sa_reheat_threshold;
    ITERATIONS_PER_CALL = g_config.sa_iterations_per_call;
    iterations_without_improvement = 0;
}

static int find_best_charging_station(int from_node, int to_node, double energy_at_from)
{
    int best_cs = -1;
    // We now look for the maximum "score" instead of the minimum detour.
    double max_score = -DBL_MAX;

    // A single loop to check all charging stations, including the depot.
    for (int cs_id = 0; cs_id < ACTUAL_PROBLEM_SIZE; cs_id++)
    {

        if (is_charging_station(cs_id))
        {
            if (cs_id == from_node)
                continue;
            // Check if we can reach the station from our current location
            if (get_energy_consumption(from_node, cs_id) <= energy_at_from)
            {
                // Check if we can reach the final destination AFTER charging at the station
                if (BATTERY_CAPACITY >= get_energy_consumption(cs_id, to_node))
                {
                    // --- NEW "LOOK-AHEAD" LOGIC ---
                    // 1. Calculate the penalty (the detour distance)
                    double detour_penalty = get_distance(from_node, cs_id) + get_distance(cs_id, to_node) - get_distance(from_node, to_node);

                    // 2. Calculate the reward (how much energy we have left at the *next* customer)
                    double energy_reward = BATTERY_CAPACITY - get_energy_consumption(cs_id, to_node);

                    // 3. The best station is one that has a high reward and a low penalty.
                    double score = energy_reward - detour_penalty;

                    if (score > max_score)
                    {
                        max_score = score;
                        best_cs = cs_id;
                    }
                }
            }
        }
    }
    return best_cs;
}

// ADD THIS MISSING FUNCTION
double get_solution_cost(int *tour, int &size)
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

static void create_neighbor_solution(int *neighbor_buffer, const int *tour, int &size, std::mt19937 &generator)
{
    // Get operator weights from the global configuration
    int w_ruin_recreate, w_swap, w_remove_station;
    double temp_ratio = T / initial_temperature;

    if (temp_ratio > g_config.sa_adaptive_threshold)
    {
        // HOT PHASE (Exploration): Use weights as defined in the config file.
        w_ruin_recreate = g_config.sa_weight_ruin_recreate;
        w_swap = g_config.sa_weight_swap;
        w_remove_station = g_config.sa_weight_remove_station;
    }
    else
    {
        // COOL PHASE (Exploitation): Invert the weights of the main operators.
        // Swap becomes the dominant operator for fine-tuning.
        w_ruin_recreate = g_config.sa_weight_swap; // Use swap's weight
        w_swap = g_config.sa_weight_ruin_recreate; // Use R&R's weight
        w_remove_station = g_config.sa_weight_remove_station; // Stays the same
    }

    int total_weight = w_ruin_recreate + w_swap + w_remove_station;

    // If all weights are zero, default to one operator to avoid errors.
    if (total_weight == 0)
    {
        ruin_and_recreate_operator(neighbor_buffer, tour, size, generator);
        return;
    }

    // Pick a random number in the range of the total weight
    std::uniform_int_distribution<int> dist(1, total_weight);
    int p = dist(generator);

    // Select the operator based on the random number and weights
    if (p <= w_ruin_recreate)
    {
        ruin_and_recreate_operator(neighbor_buffer, tour, size, generator);
    }
    else if (p <= w_ruin_recreate + w_swap)
    {
        swap_operator(neighbor_buffer, tour, size, generator);
    }
    else
    {
        remove_station_operator(neighbor_buffer, tour, size, generator);
    }
}

void run_heuristic()
{
    for (int i = 0; i < ITERATIONS_PER_CALL; ++i)
    {
        int neighbor_tour_size = current_tour_size;

        create_neighbor_solution(neighbor_tour, current_tour, neighbor_tour_size, g);

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
                record_fitness(get_evals(), neighbor_energy);
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
                    record_fitness(get_evals(), neighbor_energy);
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
    // if (iterations_without_improvement >= REHEAT_ITERATION_THRESHOLD)
    // {
    //     T = initial_temperature * 0.6;      // Reheat to a high temperature
    //     iterations_without_improvement = 0; // Reset the counter
    // }
}

void free_heuristic()
{
    if (best_sol != nullptr)
    {
        delete[] best_sol->tour;
        delete best_sol;
        best_sol = nullptr;
    }
    if (repair_buffer != nullptr)
    {
        delete[] repair_buffer;
        repair_buffer = nullptr;
    }
    if (g_nearest_station_energy_cache != nullptr)
    {
        delete[] g_nearest_station_energy_cache;
        g_nearest_station_energy_cache = nullptr;
    }
    if (current_tour != nullptr)
        delete[] current_tour;
    current_tour = nullptr;
    if (neighbor_tour != nullptr)
        delete[] neighbor_tour;
    neighbor_tour = nullptr;
}