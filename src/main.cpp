#include<iostream>
#include<stdlib.h>
#include<limits.h>

#include "EVRP.hpp"
#include "heuristic.hpp"
#include "stats.hpp"

using namespace std;


/*initialiazes a run for your heuristic*/
void start_run(int r){

  srand(r); //random seed
  init_current_best();
  cout << "Run: " << r << " with random seed " << r << endl;
}

/*gets an observation of the run for your heuristic*/
void end_run(int r){
  get_mean(r-1,get_current_best()); //from stats.h
  cout << "End of run " << r << " with best solution quality " << get_current_best() << " total evaluations: " << get_evals()  << endl;
  cout << " " << endl;
}

/*sets the termination conidition for your heuristic*/
bool termination_condition(void) {
 
  bool flag; 
  if(get_evals() >= TERMINATION) 
    flag = true;
  else
    flag = false;

  return flag;
}


/****************************************************************/
/*                Main Function                                 */
/****************************************************************/
int main(int argc, char *argv[]) {

    int run;
    /*Step 1*/
    problem_instance = argv[1];      //pass the .evrp filename as an argument
    read_problem(problem_instance);  //Read EVRP from file from EVRP.h

    /*Step 2*/
    open_stats(); //open text files to store the best values from the 20 runs stats.h
    init_evals(); // Initialize the counter once before all runs
    for(run = 1; run <= MAX_TRIALS; run++){
    /*Step 3*/
    start_run(run);

    //Initialize your heuristic here
    initialize_heuristic(); //heuristic.h

    /*Step 4*/
    // We will use our own simple termination condition.
// This runs the heuristic 25000 times, which is a reasonable number of evaluations.
for (int i = 0; i < 25000; i++) {
    run_heuristic();
}

    /*Step 5*/
    end_run(run);  //store the best solution quality for each run
}
    
    /*Step 6*/
    // This is now the single, correct call to close_stats.
    // We pass MAX_TRIALS to ensure it has a valid run number for saving the final tour.
    close_stats(MAX_TRIALS); 

    //free memory
    free_stats();
    free_heuristic();
    free_EVRP();

    return 0;
}
