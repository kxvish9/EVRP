#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP
#include<random>
struct solution{
  int *tour;
  int id;
  double tour_length;
  int steps;
};

extern solution *best_sol;

void initialize_heuristic();
void run_heuristic();
void free_heuristic();
const char* get_base_filename(const char* path);
static int* create_neighbor_solution(const int* tour, int tour_size, int& new_size, std::mt19937& generator);
static double get_solution_cost(int *tour, int &size);
#endif // HEURISTIC_HPP