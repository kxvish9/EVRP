#include<cmath>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<cstring>
#include<math.h>
#include<fstream>
#include<time.h>
#include<limits.h>
#include <direct.h> // For _mkdir on Windows
#include <filesystem>
#include "EVRP.hpp"
#include "stats.hpp"
#include "heuristic.hpp"
using namespace std;

//Used to output offline performance and population diversity

FILE *log_performance;
//output files
char *perf_filename;

double* perf_of_trials;
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
void open_stats(void) {
    // Debug message to confirm the function is called
    printf("DEBUG: open_stats() function called.\n");

    // Initialize performance tracker
    perf_of_trials = new double[MAX_TRIALS];
    for (int i = 0; i < MAX_TRIALS; i++) {
        perf_of_trials[i] = 0.0;
    }

    // Create the 'stats' directory if it doesn't exist
    const char* base_name = get_base_filename(problem_instance);
    char stats_dir[CHAR_LEN];
    sprintf(stats_dir, "stats/%s", base_name);
    _mkdir("stats");
    _mkdir(stats_dir); // Create the problem-specific subdirectory

    perf_filename = new char[CHAR_LEN];
    sprintf(perf_filename, "%s/results.txt", stats_dir);

    // Open the file in "write" mode ("w") to overwrite it each time
    if ((log_performance = fopen(perf_filename, "w")) == NULL) {
        printf("DEBUG: ERROR - Could not open %s\n", perf_filename);
        exit(2);
    } else {
        printf("DEBUG: %s opened successfully.\n", perf_filename);
    }
}


void get_mean(int r, double value) {

  perf_of_trials[r] = value;
  // Save the tour at the end of each run
    const char* base_name = get_base_filename(problem_instance);
    save_tour(best_sol, base_name, r + 1); // r is 0-indexed, so we add 1 for file naming
}


double mean(double* values, int size){
  int i;
  double m = 0.0;
  for (i = 0; i < size; i++){
      m += values[i];
  }
  m = m / (double)size;
  return m; //mean
}

double stdev(double* values, int size, double average){
  int i;
  double dev = 0.0;

  if (size <= 1)
    return 0.0;
  for (i = 0; i < size; i++){
    dev += ((double)values[i] - average) * ((double)values[i] - average);
  }
  return sqrt(dev / (double)(size - 1)); //standard deviation
}

double best_of_vector(double *values, int l ) {
  double min;
  int k;
  k = 0;
  min = values[k];
  for( k = 1 ; k < l ; k++ ) {
    if( values[k] < min ) {
      min = values[k];
    }
  }
  return min;
}


double worst_of_vector(double *values, int l ) {
  double max;
  int k;
  k = 0;
  max = values[k];
  for( k = 1 ; k < l ; k++ ) {
    if( values[k] > max ){
      max = values[k];
    }
  }
  return max;
}
void save_tour(solution* sol, const char* filename_prefix, int run_number) {
    // Create a directory for the tour files if it doesn't exist
    _mkdir("tours");

    // Create the problem-specific subdirectory
    char tour_dir[CHAR_LEN];
    sprintf(tour_dir, "tours/%s", filename_prefix);
    _mkdir(tour_dir);

    // Create the full filename
    char tour_filename[CHAR_LEN];
    sprintf(tour_filename, "%s/run-%d.tour", tour_dir, run_number);

    FILE* tour_file = fopen(tour_filename, "w");
    if (tour_file == NULL) {
        printf("ERROR: Could not open tour file %s\n", tour_filename);
        return;
    }

    // Write the tour to the file
    for (int i = 0; i < sol->steps; i++) {
        fprintf(tour_file, "%d ", sol->tour[i]);
    }

    fclose(tour_file);
    // This line is optional, you can remove it if you don't want the console message
    // printf("Tour for run %d saved to %s\n", run_number, tour_filename); 
}


void close_stats(int run) {
    double perf_mean_value = mean(perf_of_trials, MAX_TRIALS);
    double perf_stdev_value = stdev(perf_of_trials, MAX_TRIALS, perf_mean_value);

    // Open the file one last time to write summary stats
    if ((log_performance = fopen(perf_filename, "w")) != NULL) {
        fprintf(log_performance, "Mean %f\n", perf_mean_value);
        fprintf(log_performance, "StDev %f\n", perf_stdev_value);
        fprintf(log_performance, "Min %f\n", best_of_vector(perf_of_trials, MAX_TRIALS));
        fprintf(log_performance, "Max %f\n", worst_of_vector(perf_of_trials, MAX_TRIALS));
        fprintf(log_performance, "\n--- Raw Run Data ---\n");
        for (int i = 0; i < MAX_TRIALS; i++) {
            fprintf(log_performance, "Run %d: %.2f\n", i + 1, perf_of_trials[i]);
        }
        fclose(log_performance);
    } else {
        printf("ERROR: Could not open stats file for final write.\n");
    }

    // Save the final tour from the last run for visualization

}


void free_stats(){

  //free memory
  delete[] perf_of_trials;
  delete[] perf_filename;
}


