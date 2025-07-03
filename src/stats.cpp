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
    mkdir("stats");

    // Prepare filename for the output file
    perf_filename = new char[CHAR_LEN];
    const char* base_name = get_base_filename(problem_instance);
    sprintf(perf_filename, "stats/%s.txt", base_name);

    // Open the output file and check for errors
    if ((log_performance = fopen(perf_filename, "a")) == NULL) {
        printf("DEBUG: ERROR - Could not open %s\n", perf_filename);
        exit(2);
    } else {
        printf("DEBUG: %s opened successfully.\n", perf_filename);
    }
}


void get_mean(int r, double value) {

  perf_of_trials[r] = value;

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
void save_tour(const char* filename_prefix, int run_number) {
    // Create a directory for the tour files if it doesn't exist
    _mkdir("tours");

    // Create the full filename
    char tour_filename[CHAR_LEN];
    sprintf(tour_filename, "tours/run-%d-%s.tour", run_number, filename_prefix);

    FILE* tour_file = fopen(tour_filename, "w");
    if (tour_file == NULL) {
        printf("ERROR: Could not open tour file %s\n", tour_filename);
        return;
    }

    // Write the tour to the file
    for (int i = 0; i < best_sol->steps; i++) {
        fprintf(tour_file, "%d ", best_sol->tour[i]);
    }

    fclose(tour_file);
    printf("Tour for run %d saved to %s\n", run_number, tour_filename);
}


void close_stats(int run){
  int i,j;
  double perf_mean_value, perf_stdev_value;
 
  //For statistics
  for(i = 0; i < MAX_TRIALS; i++){
    //cout << i << " " << perf_of_trials[i] << endl;
    //cout << i << " " << time_of_trials[i] << endl;
    fprintf(log_performance, "%.2f", perf_of_trials[i]);
    fprintf(log_performance,"\n");

  }

  perf_mean_value = mean(perf_of_trials,MAX_TRIALS);
  perf_stdev_value = stdev(perf_of_trials,MAX_TRIALS,perf_mean_value);
  fprintf(log_performance,"Mean %f\t ",perf_mean_value);
  fprintf(log_performance,"\tStd Dev %f\t ",perf_stdev_value);
  fprintf(log_performance,"\n");
  fprintf(log_performance, "Min: %f\t ", best_of_vector(perf_of_trials,MAX_TRIALS));
  fprintf(log_performance,"\n");
  fprintf(log_performance, "Max: %f\t ", worst_of_vector(perf_of_trials,MAX_TRIALS));
  fprintf(log_performance,"\n");
  const char* base_name = get_base_filename(problem_instance);
  save_tour(base_name, run);

  fclose(log_performance);
 

}


void free_stats(){

  //free memory
  delete[] perf_of_trials;
  delete[] perf_filename;
}


