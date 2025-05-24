from flask import Flask, request, jsonify
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import requests
import os

app = Flask(__name__)

ORS_API_KEY = 5b3ce3597851110001cf6248f3380fa418534bd499a9945c9361973e
ORS_MATRIX_URL = "https://api.openrouteservice.org/v2/matrix/driving-car"

def get_ors_distance_matrix(coords):
    headers = {
        "Authorization": ORS_API_KEY,
        "Content-Type": "application/json"
    }
    body = {
        "locations": [[lon, lat] for lat, lon in coords],  # ORS expects [lon, lat]
        "metrics": ["distance"],  # can also use "duration"
        "units": "m"
    }

    response = requests.post(ORS_MATRIX_URL, json=body, headers=headers)
    if response.status_code != 200:
        raise Exception(f"ORS API error: {response.status_code} - {response.text}")

    return response.json()["distances"]

@app.route("/optimize", methods=["POST"])
def optimize():
    data = request.get_json()
    raw_locations = data["locations"]

    coords = [
        (float(loc["lat"]), float(loc["lng"]))
        for loc in raw_locations
        if loc.get("lat") not in [None, ""] and loc.get("lng") not in [None, ""]
    ]

    if len(coords) < 2:
        return jsonify({"error": "Legalább 2 érvényes koordináta szükséges."}), 400

    try:
        distance_matrix = get_ors_distance_matrix(coords)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    # OR-Tools
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_callback(from_idx, to_idx):
        return int(distance_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)])

    transit_index = routing.RegisterTransitCallback(dist_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    # No return to depot
    routing.SetFixedCostOfAllVehicles(0)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)

    route = []
    if solution:
        index = routing.Start(0)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))  # optional: remove if no return

    return jsonify({"route": route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
