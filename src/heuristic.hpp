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

static void swap_operator(int *neighbor_buffer, const int *tour, int &size, std::mt19937 &generator);
void initialize_heuristic();
void run_heuristic();
void free_heuristic();
const char* get_base_filename(const char* path);
static void create_neighbor_solution(int *neighbor_buffer, const int *tour, int &size, std::mt19937 &generator);
double get_solution_cost(int *tour, int &size);
static int find_best_charging_station(int from_node, int to_node, double energy_at_from);
void print_operator_stats();
#endif // HEURISTIC_HPP