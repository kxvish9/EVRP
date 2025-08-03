#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

// Enum for algorithm choice
enum Algorithm
{
    SA // Simulated Annealing
    // Add other algorithms here in the future, e.g., GA, ACO
};

// Enum for SA operator choice
enum SA_Operator
{
    TWO_OPT,
    SWAP,
    RUIN_RECREATE
};

struct Config
{
    // General settings
    std::string algorithm_name;
    Algorithm algorithm;
    int trial_runs;
    long termination_evals_factor;

    // SA specific settings
    std::string sa_operator_name;
    SA_Operator sa_operator;
    double sa_initial_temp;
    double sa_cooling_rate;
    int sa_reheat_threshold;
    int sa_iterations_per_call;
    // Penalty weights
    double penalty_weight_energy;
    double penalty_weight_capacity;
};

// Global config object
extern Config g_config;

// Function to load settings from a file
void load_config(const std::string &filename);

#endif // CONFIG_HPP