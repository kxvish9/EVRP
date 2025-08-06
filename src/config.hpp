#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

// Enum for algorithm choice
enum Algorithm
{
    SA // Simulated Annealing
    // Add other algorithms here in the future, e.g., GA, ACO
};

// This enum is no longer needed by the config, but heuristic might still use it internally.
// We can remove it if it's fully replaced. For now, let's leave it.


struct Config
{
    // General settings
    std::string algorithm_name;
    Algorithm algorithm;
    int trial_runs;
    long termination_evals_factor;

    // SA specific settings
    // REMOVED: sa_operator_name and sa_operator
    // std::string sa_operator_name;
    // SA_Operator sa_operator;
    
    // ADDED: Operator weights
    int sa_weight_ruin_recreate;
    int sa_weight_swap;
    int sa_weight_remove_station;

    double sa_initial_temp;
    double sa_cooling_rate;
    double sa_reheat_threshold;
    int sa_iterations_per_call;
    double sa_adaptive_threshold;
};

// Global config object
extern Config g_config;

// Function to load settings from a file
void load_config(const std::string &filename);

#endif // CONFIG_HPP