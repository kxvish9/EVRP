#include "heuristic.hpp"
#define MAX_TRIALS 	20 									//DO NOT CHANGE THE NUMBER
#define CHAR_LEN 100
#ifndef STATS_HPP
#define STATS_HPP
void open_stats(void);									//creates the output file
void close_stats(int run);								//stores the best values for each run
void get_mean(int r, double value);						//stores the observation from each run
void free_stats();										//free memory
#endif // STATS_HPP