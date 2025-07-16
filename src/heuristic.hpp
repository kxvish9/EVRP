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
const char* get_base_filename(const char* path);
#endif // HEURISTIC_HPP