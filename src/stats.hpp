#include "heuristic.hpp"
#define MAX_TRIALS 	21 					
#define CHAR_LEN 100
#ifndef STATS_HPP
#define STATS_HPP
void open_stats(void);					
void close_stats(void);			
// Records the performance of a single run
void get_mean(int r, double value);
// Saves the tour from a single run to a file
void record_fitness(double eval, double fitness);
void save_tour(solution* sol, int run_number);
void free_stats();	
void start_run(int r);
void end_run(int r);							
#endif // STATS_HPP