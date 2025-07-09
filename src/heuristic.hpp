#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

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
int get_current_tour_size_debug();
#endif // HEURISTIC_HPP