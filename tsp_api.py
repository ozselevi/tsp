from flask import Flask, request, jsonify
import googlemaps
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import os

app = Flask(__name__)

gmaps = googlemaps.Client(key=os.environ["GOOGLE_API_KEY"])

@app.route("/optimize", methods=["POST"])
def optimize():
    data = request.get_json()
    locations = data["locations"]
    coords = [(loc["lat"], loc["lng"]) for loc in locations]

    matrix_result = gmaps.distance_matrix(coords, coords, mode="driving")
    distance_matrix = [
        [elem["distance"]["value"] for elem in row["elements"]]
        for row in matrix_result["rows"]
    ]

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
        route.append(manager.IndexToNode(index))

    return jsonify({"route": route})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
