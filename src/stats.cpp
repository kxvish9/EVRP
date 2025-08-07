#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <direct.h> // For _mkdir on Windows
#include "EVRP.hpp"
#include "stats.hpp"
#include "heuristic.hpp"
#include "config.hpp"
using namespace std;

// Used to output offline performance and population diversity

FILE *log_performance;
// output files
char *perf_filename;
FILE *log_fitness_trace;
char *fitness_trace_filename;
double *perf_of_trials;
// Helper to create a directory for a given filepath
void create_directory_for_file(const char *filepath)
{
  char *path_copy = new char[strlen(filepath) + 1];
  strcpy(path_copy, filepath);
  // In C++, std::filesystem::path(path_copy).parent_path() is safer
  // but for C-style strings, we find the last slash.
  char *last_slash = strrchr(path_copy, '/');
  if (last_slash != NULL)
  {
    *last_slash = '\0'; // Cut the string at the last slash
    _mkdir(path_copy);  // Create the directory
  }
  delete[] path_copy;
}
/*
 * Sets up the initial state for a single trial run.
 */
void start_run(int r)
{
  init_evals();
  init_current_best();
  cout << "Run: " << r << " with random seed " << r << endl;
  // Create and open the fitness trace file for the current run
  const char *base_name = get_base_filename(problem_instance);
  sprintf(fitness_trace_filename, "stats/%s/fitness_run_%d.txt", base_name, r);
  create_directory_for_file(fitness_trace_filename);
  if ((log_fitness_trace = fopen(fitness_trace_filename, "w")) == NULL)
  {
    exit(2);
  }
  // Add a header for easy parsing later
  fprintf(log_fitness_trace, "Evaluations\tFitness\n");
}

/*
 * Gathers and prints results from a single trial run.
 */
void end_run(int r)
{
  // First, record the performance value for the run
  get_mean(r - 1, get_current_best());
  // Second, save the best tour found in that run
  save_tour(best_sol, r);
  if (log_fitness_trace != NULL)
  {
    fclose(log_fitness_trace);
  }
  cout << "End of run " << r << " with best solution quality " << get_current_best() << endl;
  cout << " " << endl;
  print_operator_stats();
}
void open_stats(void)
{
  // Initialize performance tracker
  perf_of_trials = new double[g_config.trial_runs];
  for (int i = 0; i < g_config.trial_runs; i++)
  {
    perf_of_trials[i] = 0.0;
  }

  // Prepare the full path for the results file
  const char *base_name = get_base_filename(problem_instance);
  perf_filename = new char[CHAR_LEN];
  fitness_trace_filename = new char[CHAR_LEN];
  sprintf(perf_filename, "stats/%s/results.txt", base_name);

  // Use the helper to create the necessary directories
  create_directory_for_file(perf_filename);

  // Open the file in "write" mode, which overwrites previous results
  if ((log_performance = fopen(perf_filename, "w")) == NULL)
  {
    exit(2);
  }
}

void get_mean(int r, double value)
{
  perf_of_trials[r] = value;
}

double mean(double *values, int size)
{
  int i;
  double m = 0.0;
  for (i = 0; i < size; i++)
  {
    m += values[i];
  }
  m = m / (double)size;
  return m; // mean
}

double stdev(double *values, int size, double average)
{
  int i;
  double dev = 0.0;

  if (size <= 1)
    return 0.0;
  for (i = 0; i < size; i++)
  {
    dev += ((double)values[i] - average) * ((double)values[i] - average);
  }
  return sqrt(dev / (double)(size - 1)); // standard deviation
}

double best_of_vector(double *values, int l)
{
  double min;
  int k;
  k = 0;
  min = values[k];
  for (k = 1; k < l; k++)
  {
    if (values[k] < min)
    {
      min = values[k];
    }
  }
  return min;
}

double worst_of_vector(double *values, int l)
{
  double max;
  int k;
  k = 0;
  max = values[k];
  for (k = 1; k < l; k++)
  {
    if (values[k] > max)
    {
      max = values[k];
    }
  }
  return max;
}
void save_tour(solution *sol, int run_number)
{
  const char *base_name = get_base_filename(problem_instance);
  char tour_filename[CHAR_LEN];
  sprintf(tour_filename, "tours/%s/run-%d.tour", base_name, run_number);

  create_directory_for_file(tour_filename);
  FILE *tour_file = fopen(tour_filename, "w");
  if (tour_file == NULL)
  {
    return; // Silently fail if file can't be opened
  }

  for (int i = 0; i < sol->steps; i++)
  {
    fprintf(tour_file, "%d ", sol->tour[i]);
  }
  fclose(tour_file);
}

void close_stats(void)
{
  double perf_mean_value = mean(perf_of_trials, g_config.trial_runs);
  double perf_stdev_value = stdev(perf_of_trials, g_config.trial_runs, perf_mean_value);

  // Open the file one last time to write summary stats
  if ((log_performance = fopen(perf_filename, "w")) != NULL)
  {
    fprintf(log_performance, "Mean %f\n", perf_mean_value);
    fprintf(log_performance, "StDev %f\n", perf_stdev_value);
    fprintf(log_performance, "Min %f\n", best_of_vector(perf_of_trials, g_config.trial_runs));
    fprintf(log_performance, "Max %f\n", worst_of_vector(perf_of_trials, g_config.trial_runs));
    fprintf(log_performance, "\n--- Raw Run Data ---\n");
    for (int i = 0; i < g_config.trial_runs; i++)
    {
      fprintf(log_performance, "Run %d: %.2f\n", i + 1, perf_of_trials[i]);
    }
    fclose(log_performance);
  }
  else
  {
    printf("ERROR: Could not open stats file for final write.\n");
  }

  // Save the final tour from the last run for visualization
}
void record_fitness(double eval, double fitness)
{
  if (log_fitness_trace != NULL)
  {
    fprintf(log_fitness_trace, "%f\t%f\n", eval, fitness);
  }
}
void free_stats()
{

  // free memory
  delete[] fitness_trace_filename;
  delete[] perf_of_trials;
  delete[] perf_filename;
}