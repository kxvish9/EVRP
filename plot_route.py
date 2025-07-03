import matplotlib.pyplot as plt
import sys
import os

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

    # Read coordinates
    for i in range(coord_start, demand_start - 1):
        parts = lines[i].strip().split()
        node_id, x, y = int(parts[0]), float(parts[1]), float(parts[2])
        nodes[node_id] = {'x': x, 'y': y, 'type': 'customer'}

    # Identify depot
    nodes[1]['type'] = 'depot'

    # Correctly handle STATIONS_COORD_SECTION
    try:
        # Use the correct section header from your file
        station_start = lines.index("STATIONS_COORD_SECTION") + 1
        for i in range(station_start, len(lines)):
            parts = lines[i].strip().split()
            if not parts or not parts[0].isdigit():
                break # Stop if we hit a non-station line like DEPOT_SECTION
            station_id = int(parts[0])
            if station_id in nodes:
                nodes[station_id]['type'] = 'station'
    except ValueError:
        print("Notice: No STATIONS_COORD_SECTION found in this problem file.")
            
    return nodes

def parse_tour_file(filepath):
    """Parses the .tour file, ignoring any '0' values."""
    with open(filepath, 'r') as f:
        tour_str = f.read().strip().split()
    return [int(node) for node in tour_str if int(node) != 0]

def plot_evrp_route(problem_file, tour_file):
    """Plots the EVRP route."""
    nodes = parse_evrp_file(problem_file)
    tour = parse_tour_file(tour_file)

    plt.figure(figsize=(12, 8))

    for node_id, data in nodes.items():
        if data['type'] == 'depot':
            plt.plot(data['x'], data['y'], 'ks', markersize=12, label='Depot')
        elif data['type'] == 'station':
            plt.plot(data['x'], data['y'], 'g^', markersize=10, label='Station' if 'Station' not in plt.gca().get_legend_handles_labels()[1] else "")
        else:
            plt.plot(data['x'], data['y'], 'bo', markersize=8, label='Customer' if 'Customer' not in plt.gca().get_legend_handles_labels()[1] else "")
        plt.text(data['x'], data['y'] + 1.5, str(node_id), fontsize=9, ha='center')

    sub_routes = []
    current_route = []
    if tour:
        if tour[0] != 1:
            tour.insert(0, 1)
        for node_id in tour:
            current_route.append(node_id)
            if node_id == 1 and len(current_route) > 1:
                sub_routes.append(current_route)
                current_route = [1]
        if len(current_route) > 1:
            sub_routes.append(current_route)
            
    for i, route in enumerate(sub_routes):
        route_x = [nodes[node_id]['x'] for node_id in route]
        route_y = [nodes[node_id]['y'] for node_id in route]
        color = plt.cm.viridis(i / max(1, len(sub_routes)))
        plt.plot(route_x, route_y, color=color, linestyle='-', marker='o', markersize=3)

    plt.title(f'EVRP Solution for {os.path.basename(problem_file)}')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    
    plot_filename = os.path.splitext(os.path.basename(tour_file))[0] + '.png'
    save_path = os.path.join('../plots', plot_filename)
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path)
    print(f"Plot saved to {save_path}")
    # plt.show() # This line is removed to prevent the script from hanging

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python plot_route.py <path_to_problem_file> <path_to_tour_file>")
        sys.exit(1)
        
    problem_file = sys.argv[1]
    tour_file = sys.argv[2]
    
    if not os.path.exists(problem_file):
        print(f"Error: Problem file not found at {problem_file}")
        sys.exit(1)

    if not os.path.exists(tour_file):
        print(f"Error: Tour file not found at {tour_file}")
        sys.exit(1)

    plot_evrp_route(problem_file, tour_file)