#include <iostream>
#include <cstdlib>
#include <ctime> // Required for time()
#include <limits.h>
#include <cfloat>
#include "EVRP.hpp"
#include "heuristic.hpp"
#include "stats.hpp"

using namespace std;

int main(int argc, char *argv[])
{
  // Safety check for command-line arguments
  if (argc != 2)
  {
    cout << "Usage: ./main <problem_instance>" << endl;
    return 1;
  }
  // Variables to track the best solution found across all runs
  int best_run = -1;
  double best_run_fitness = DBL_MAX;
  char best_tour_filename[CHAR_LEN];
  int run;
  srand(time(NULL)); // Seed the C-style random generator once

  // Step 1: Read the problem instance from file
  problem_instance = argv[1];
  read_problem(problem_instance);

  // Step 2: Prepare for statistics collection
  open_stats();

  // Step 3: Perform all trial runs
  for (run = 1; run <= MAX_TRIALS; run++)
  {
    start_run(run);
    initialize_heuristic();

    // This fixed loop executes the heuristic for a set number of iterations
    for (int i = 0; i < 25000; i++)
    {
      run_heuristic();
    }

    end_run(run);
    // Check if the current run is the best one so far
    if (best_sol->tour_length < best_run_fitness)
    {
      best_run_fitness = best_sol->tour_length;
      best_run = run;
    }
  }

  // Step 4: Finalize stats and save the last tour
  close_stats();
  // --- Call Python script to plot the best tour ---
  printf("\nPlotting best tour from Run %d...\n", best_run);

  // Construct the path to the tour file of the best run
  const char *base_name = get_base_filename(problem_instance);
  sprintf(best_tour_filename, "tours/%s/run-%d.tour", base_name, best_run);

  // Construct the full python command
  char command[CHAR_LEN * 3];
  sprintf(command, "python ../plot_route.py %s %s", problem_instance, best_tour_filename);

  // Execute the command
  system(command);

  // Step 5: Free all allocated memory
  free_stats();
  free_heuristic();
  free_EVRP();

  return 0;
}
