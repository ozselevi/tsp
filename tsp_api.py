from flask import Flask, request, jsonify
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from math import radians, sin, cos, sqrt, atan2

app = Flask(__name__)

def haversine_distance(coord1, coord2):
    # Koordináták (lat, lon)
    lat1, lon1 = coord1
    lat2, lon2 = coord2

    # Föld sugara km-ben
    R = 6371.0

    # Radiánba váltás
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)

    a = sin(dphi / 2)**2 + cos(phi1) * cos(phi2) * sin(dlambda / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return R * c

@app.route("/optimize", methods=["POST"])
def optimize():
    data = request.get_json()
    raw_locations = data["locations"]

    # Tisztítás: konvertál float-ra és szűrés, ha hiányzik adat
    coords = [
        (float(loc["lat"]), float(loc["lng"]))
        for loc in raw_locations
        if loc.get("lat") not in [None, ""] and loc.get("lng") not in [None, ""]
    ]

    if len(coords) < 2:
        return jsonify({"error": "Legalább 2 érvényes koordináta szükséges."}), 400

    # Távolságmátrix (km-ben)
    distance_matrix = [
        [int(haversine_distance(c1, c2) * 1000) for c2 in coords]
        for c1 in coords
    ]

    # OR-Tools
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_callback(from_idx, to_idx):
        return distance_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]

    transit_index = routing.RegisterTransitCallback(dist_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(params)

    route = []
    if solution:
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))  # Visszatérés a kiindulóponthoz

    return jsonify({"route": route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
