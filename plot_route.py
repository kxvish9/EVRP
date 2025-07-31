import matplotlib.pyplot as plt
import sys
import os
import pandas as pd
def parse_evrp_file(filepath):
    """Parses the .evrp file to get node coordinates and types."""
    nodes = {}
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f.readlines()]
    
    try:
        coord_start = lines.index("NODE_COORD_SECTION") + 1
        demand_start = lines.index("DEMAND_SECTION") + 1
    except ValueError as e:
        print(f"Error: A required section was not found - {e}")
        sys.exit(1)

    for i in range(coord_start, demand_start - 1):
        parts = lines[i].strip().split()
        node_id, x, y = int(parts[0]), float(parts[1]), float(parts[2])
        nodes[node_id] = {'x': x, 'y': y, 'type': 'customer'}

    nodes[1]['type'] = 'depot'

    try:
        station_start = lines.index("STATIONS_COORD_SECTION") + 1
        for i in range(station_start, len(lines)):
            parts = lines[i].strip().split()
            if not parts or not parts[0].isdigit():
                break
            station_id = int(parts[0])
            if station_id in nodes:
                nodes[station_id]['type'] = 'station'
    except ValueError:
        print("Notice: No STATIONS_COORD_SECTION found in this problem file.")
            
    return nodes

def parse_tour_file(filepath, num_vehicles):
    """
    Parses the .tour file and splits it into sub-routes for a specific number of vehicles.
    """
    with open(filepath, 'r') as f:
        tour_str_list = f.read().strip().split()

    # This single line robustly converts all occurrences of '0' to 1
    # before converting the rest of the nodes to integers. This prevents any '0'
    # from ever entering the tour list.
    full_tour = [1 if node == '0' else int(node) for node in tour_str_list]
    
    sub_routes = []
    start_idx = 0

    # The first (k-1) vehicles get one trip each, ending at the first depot they encounter.
    for i in range(num_vehicles - 1):
        try:
            # Find the next depot occurrence to define the end of the current route
            end_idx = full_tour.index(1, start_idx + 1)
            sub_routes.append(full_tour[start_idx : end_idx + 1])
            start_idx = end_idx
        except ValueError:
            # This case handles if the tour file implies fewer vehicles than specified.
            print(f"Warning: Not enough depot visits in tour to support {num_vehicles} vehicles. Assigning remaining tour to last vehicle.")
            break
            
    # The last vehicle takes all the remaining parts of the tour.
    sub_routes.append(full_tour[start_idx:])
            
    return sub_routes

def plot_evrp_route(problem_file, tour_file, num_vehicles):
    """Plots the EVRP solution with a specified number of routes."""
    nodes = parse_evrp_file(problem_file)
    sub_routes = parse_tour_file(tour_file, num_vehicles)

    plt.figure(figsize=(14, 10))

    # Plot all nodes first
    for node_id, data in nodes.items():
        if data['type'] == 'depot':
            plt.plot(data['x'], data['y'], 'ks', markersize=12, label='Depot')
        elif data['type'] == 'station':
            plt.plot(data['x'], data['y'], 'g^', markersize=10, label='Station' if 'Station' not in plt.gca().get_legend_handles_labels()[1] else "")
        else:
            plt.plot(data['x'], data['y'], 'bo', markersize=8, label='Customer' if 'Customer' not in plt.gca().get_legend_handles_labels()[1] else "")
        plt.text(data['x'], data['y'] + 1.5, str(node_id), fontsize=9, ha='center')

    # Plot each sub-route with a different color
    for i, route in enumerate(sub_routes):
        route_x = [nodes[node_id]['x'] for node_id in route]
        route_y = [nodes[node_id]['y'] for node_id in route]
        color = plt.cm.viridis(i / max(1, len(sub_routes))) # Use a colormap for distinct colors
        plt.plot(route_x, route_y, color=color, linestyle='-', marker='o', markersize=4, label=f'Route {i+1}')

    plt.title(f'EVRP Solution for {os.path.basename(problem_file)} ({num_vehicles} Vehicles)')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    
    plot_filename = os.path.splitext(os.path.basename(tour_file))[0] + '.png'
    save_path = os.path.join('../plots', os.path.splitext(os.path.basename(problem_file))[0], plot_filename)
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path)
    print(f"Plot saved to {save_path}")

def plot_fitness_curve(data_file):
    """
    Plots the fitness (tour length) over evaluations from a given data file.
    """
    if not os.path.exists(data_file):
        print(f"Error: Fitness data file not found at {data_file}")
        return

    # Read the tab-separated data
    try:
        data = pd.read_csv(data_file, sep='\t')
        if data.empty:
            print("Error: Fitness data file is empty.")
            return
    except Exception as e:
        print(f"Error reading data file: {e}")
        return
    
    # --- Plotting ---
    plt.figure(figsize=(12, 7))
    plt.plot(data.iloc[:, 0], data.iloc[:, 1], linestyle='-', color='b', label='Best Fitness')
    
    problem_name = os.path.basename(os.path.dirname(os.path.dirname(data_file)))
    run_name = os.path.splitext(os.path.basename(data_file))[0].replace('_', ' ').replace('-', ' ').title()
    
    plt.title(f'SA Fitness Convergence for {problem_name} ({run_name})')
    plt.xlabel('Fitness Evaluations')
    plt.ylabel('Best Tour Length (Fitness)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # --- Saving the Plot ---
    output_dir = os.path.join('../plots', problem_name)
    os.makedirs(output_dir, exist_ok=True)
    
    plot_filename = f"fitness_plot_{os.path.splitext(os.path.basename(data_file))[0]}.png"
    save_path = os.path.join(output_dir, plot_filename)
    
    plt.savefig(save_path)
    print(f"Fitness plot saved to {save_path}")
    plt.close()

if __name__ == "__main__":
    # Decide which function to run based on arguments
    if len(sys.argv) > 1 and sys.argv[1] == 'plot_fitness':
        if len(sys.argv) != 3:
            print("Usage: python plot_route.py plot_fitness <path_to_fitness_data_file>")
            sys.exit(1)
        plot_fitness_curve(sys.argv[2])
    
    # --- Original route plotting logic ---
    else:
        if len(sys.argv) != 4:
            print("Usage for route plotting: python plot_route.py <path_to_problem_file> <path_to_tour_file> <num_vehicles>")
            sys.exit(1)
        
        problem_file, tour_file = sys.argv[1], sys.argv[2]
        
        try:
            num_vehicles = int(sys.argv[3])
            if num_vehicles < 1: raise ValueError
        except ValueError:
            print("Error: <num_vehicles> must be a positive integer.")
            sys.exit(1)
        
        if not os.path.exists(problem_file):
            print(f"Error: Problem file not found at {problem_file}")
            sys.exit(1)

        if not os.path.exists(tour_file):
            print(f"Error: Tour file not found at {tour_file}")
            sys.exit(1)

        plot_evrp_route(problem_file, tour_file, num_vehicles)