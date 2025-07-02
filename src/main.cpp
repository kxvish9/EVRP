#include <iostream>
#include <cstdlib> // Required for srand
#include <ctime>   // Required for time

#include "EVRP.hpp"
#include "heuristic.hpp"

using namespace std;

int main(int argc, char *argv[]) {

    if(argc != 2){
        cout << "Usage: ./main <problem_instance>" << endl;
        return 1;
    }

    // Initialize random seed - THIS IS THE CRITICAL ADDITION
    srand(time(NULL));
    
    problem_instance = argv[1];
    
    read_problem(problem_instance);
    
    initialize_heuristic();
    
    run_heuristic(); // This will now run your SA algorithm
    
    cout << "Best solution quality found: " << best_sol->tour_length << endl;
    
    print_solution(best_sol->tour, best_sol->steps);
    
    check_solution(best_sol->tour, best_sol->steps);
    
    free_EVRP();
    free_heuristic();
    
    return 0;
}