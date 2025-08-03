#include "config.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <algorithm> // for std::transform

// Define the global config object
Config g_config;

// Helper to trim whitespace from a string
static std::string trim(const std::string &s)
{
    size_t first = s.find_first_not_of(" \t\n\r");
    if (std::string::npos == first)
    {
        return s;
    }
    size_t last = s.find_last_not_of(" \t\n\r");
    return s.substr(first, (last - first + 1));
}

void load_config(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open config file: " + filename);
    }

    std::string line;
    std::string key, value;
    while (std::getline(file, line))
    {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::stringstream ss(line);
        if (std::getline(ss, key, '=') && std::getline(ss, value))
        {
            key = trim(key);
            value = trim(value);

            // --- General Settings ---
            if (key == "ALGORITHM")
            {
                g_config.algorithm_name = value;
            }
            else if (key == "TRIAL_RUNS")
            {
                g_config.trial_runs = std::stoi(value);
            }
            else if (key == "TERMINATION_EVALS_FACTOR")
            {
                g_config.termination_evals_factor = std::stol(value);
            }
            // --- SA Settings ---
            else if (key == "SA_OPERATOR")
            {
                g_config.sa_operator_name = value;
            }
            else if (key == "SA_INITIAL_TEMPERATURE")
            {
                g_config.sa_initial_temp = std::stod(value);
            }
            else if (key == "SA_COOLING_RATE")
            {
                g_config.sa_cooling_rate = std::stod(value);
            }
            else if (key == "SA_REHEAT_THRESHOLD")
            {
                g_config.sa_reheat_threshold = std::stoi(value);
            }
            else if (key == "SA_ITERATIONS_PER_CALL")
            {
                g_config.sa_iterations_per_call = std::stoi(value);
            }
            // --- Penalty Weights ---
            else if (key == "PENALTY_WEIGHT_ENERGY")
            {
                g_config.penalty_weight_energy = std::stod(value);
            }
            else if (key == "PENALTY_WEIGHT_CAPACITY")
            {
                g_config.penalty_weight_capacity = std::stod(value);
            }
        }
    }

    // Post-process string values into enums
    std::transform(g_config.algorithm_name.begin(), g_config.algorithm_name.end(), g_config.algorithm_name.begin(), ::toupper);
    if (g_config.algorithm_name == "SA")
    {
        g_config.algorithm = SA;
    }
    else
    {
        throw std::runtime_error("Unknown algorithm in config file: " + g_config.algorithm_name);
    }

    std::transform(g_config.sa_operator_name.begin(), g_config.sa_operator_name.end(), g_config.sa_operator_name.begin(), ::toupper);
    if (g_config.sa_operator_name == "TWO_OPT")
    {
        g_config.sa_operator = TWO_OPT;
    }
    else if (g_config.sa_operator_name == "SWAP")
    {
        g_config.sa_operator = SWAP;
    }
    else if (g_config.sa_operator_name == "RUIN_RECREATE")
    {
        g_config.sa_operator = RUIN_RECREATE;
    }
    else
    {
        throw std::runtime_error("Unknown SA operator in config file: " + g_config.sa_operator_name);
    }

    file.close();
}