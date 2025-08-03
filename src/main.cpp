#include <iostream>
#include <cstdlib>
#include <ctime> // Required for time()
#include <limits.h>
#include <cfloat>
#include "EVRP.hpp"
#include "heuristic.hpp"
#include "stats.hpp"
#include "config.hpp"
using namespace std;
/*sets the termination conidition for your heuristic*/
bool termination_condition(void)
{
  bool flag;
  if (get_evals() >= TERMINATION)
    flag = true;
  else
    flag = false;
  return flag;
}
int main(int argc, char *argv[])
{
  // Safety check for command-line arguments
  if (argc != 3)
  {
    cout << "Usage: ./main <problem_instance> <config_file>" << endl;
    return 1;
  }
  // Variables to track the best solution found across all runs
  int best_run = -1;
  double best_run_fitness = DBL_MAX;
  char best_tour_filename[CHAR_LEN];
  int run;
  srand(time(NULL)); // Seed the C-style random generator once
  try
  {
    load_config(argv[2]);
  }
  catch (const std::runtime_error &e) 
  {
    cout << "Error loading config: " << e.what() << endl;
    return 1;
  }
  // Step 1: Read the problem instance from file
  problem_instance = argv[1];
  read_problem(problem_instance);

  // Step 2: Prepare for statistics collection
  open_stats();

  // Step 3: Perform all trial runs
  for (run = 1; run <= g_config.trial_runs; run++)
  {
    start_run(run);
    initialize_heuristic();
    while (!termination_condition())
    {
      run_heuristic();
    }
    check_solution(best_sol->tour, best_sol->steps);
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
  sprintf(command, "python ../plot_route.py %s %s %d", problem_instance, best_tour_filename, MIN_VEHICLES);

  // Execute the command
  system(command);
  // --- Call Python script to plot the fitness curve for the best run ---
  printf("Plotting fitness curve from best run (Run %d)...\n", best_run);

  // Construct the path to the fitness data file of the best run
  char fitness_data_filename[CHAR_LEN];
  sprintf(fitness_data_filename, "stats/%s/fitness_run_%d.txt", base_name, best_run);

  // Construct a new command to call the same python script with different arguments
  char fitness_command[CHAR_LEN * 3];
  sprintf(fitness_command, "python ../plot_route.py plot_fitness \"%s\"", fitness_data_filename);

  // Execute the command to generate the fitness plot
  system(fitness_command);
  // Step 5: Free all allocated memory
  free_stats();
  free_heuristic();
  free_EVRP();

  return 0;
}
