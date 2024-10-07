package io.cvdejan.graph;

import java.util.*;
import java.util.regex.Pattern;

import static io.cvdejan.graph.DijkstraAlgorithm.calculateRadiusAndDiameter;
import static io.cvdejan.graph.DijkstraAlgorithm.dijkstraShortestPath;

public class RandomDirectedGraphGenerator {
    // Function to generate a connected, directed graph using an adjacency list
    public static Map<Integer, List<Edge>> generateRandomDirectedGraph(int N, int S, int minWeight, int maxWeight) {

        if (S < N - 1 || S > N * (N - 1)) throw new IllegalArgumentException("Number of edges (S) must be between N-1 and N*(N-1).");

        Map<Integer, List<Edge>> graph = new HashMap<>();
        Random random = new Random();

        // Initialize the adjacency list
        for (int i = 0; i < N; i++)
            graph.put(i, new ArrayList<>());

        // Generate a spanning tree first to ensure the graph is connected
        List<Integer> connectedVertices = new ArrayList<>();
        connectedVertices.add(0); // Start with the first vertex

        for (int i = 1; i < N; i++) {
            // Pick a random vertex from the connected component
            int randomConnectedVertex = connectedVertices.get(random.nextInt(connectedVertices.size()));
            int weight = random.nextInt(maxWeight - minWeight + 1) + minWeight;

            // Connect the new vertex to the randomly chosen vertex with a directed edge
            addDirectedEdge(graph, randomConnectedVertex, i, weight);
            connectedVertices.add(i);  // Add this vertex to the connected component
        }

        // We now have exactly N-1 edges (a spanning tree), so we need to add (S - N + 1) extra edges
        int extraEdges = S - (N - 1);
        while (extraEdges > 0) {
            int v1 = random.nextInt(N);
            int v2 = random.nextInt(N);

            // Ensure there is no self-loop and the edge does not already exist
            if (v1 != v2 && !directedEdgeExists(graph, v1, v2)) {
                int weight = random.nextInt(maxWeight - minWeight + 1) + minWeight;
                addDirectedEdge(graph, v1, v2, weight);
                extraEdges--;
            }
        }

        return graph;
    }

    // Helper function to add a directed edge to the adjacency list
    private static void addDirectedEdge(Map<Integer, List<Edge>> graph, int from, int to, int weight) {
        graph.get(from).add(new Edge(to, weight));
    }

    // Helper function to check if a directed edge already exists between two vertices
    private static boolean directedEdgeExists(Map<Integer, List<Edge>> graph, int from, int to) {
        for (Edge edge : graph.get(from)) {
            if (edge.to == to) return true;
        }
        return false;
    }

    // Class to represent a directed edge with a destination vertex and weight
    public static class Edge {
        int to;
        int weight;

        public Edge(int to, int weight) {
            this.to = to;
            this.weight = weight;
        }

        @Override
        public String toString() {
            return "{" + "to=" + to + ", weight=" + weight + "}";
        }
    }
    public static void main(String[] args) {

        int N = 5;    // Number of vertices
        int S = 15;    // Number of directed edges (sparseness)
        int minWeight = 1;  // Minimum edge weight
        int maxWeight = 10; // Maximum edge weight

        Pattern verticesPattern = Pattern.compile("\\d+");
        Pattern edgesPattern = Pattern.compile("\\d+");

        System.out.println("Enter the number of vertices: ");
        String numOfVertices = System.console().readLine();
        if(!verticesPattern.matcher(numOfVertices).matches()) throw new RuntimeException("Number of vertices must be digits only, equal or greater than 2!");

        N = Integer.parseInt(numOfVertices);
        if(N<2) throw new RuntimeException("Number of vertices must be digits only, equal or greater than 2!");

        System.out.println("Enter the number of directed edges (between " + (N-1) + " and " + N*(N-1) + "): ");

        String numOfEdges = System.console().readLine();
        if(!edgesPattern.matcher(numOfEdges).matches()) throw new RuntimeException("Number of edges must be digits only !");

        S = Integer.parseInt(numOfEdges);

        Map<Integer, List<Edge>> graph = generateRandomDirectedGraph(N, S, minWeight, maxWeight);

        // Print the generated graph
        for (int vertex : graph.keySet()) {
            System.out.println("Vertex " + vertex + ": " + graph.get(vertex));
        }

        calculateRadiusAndDiameter(graph);

        // Randomly select two vertices to find the shortest path between them
        Random random = new Random();
        int startVertex = random.nextInt(N);
        int endVertex = random.nextInt(N);

        System.out.println("\nFinding shortest path between Vertex " + startVertex + " and Vertex " + endVertex);

        List<Integer> shortestPath = dijkstraShortestPath(graph, startVertex, endVertex);

        if (shortestPath.isEmpty()) {
            System.out.println("No path found between Vertex " + startVertex + " and Vertex " + endVertex);
        } else {
            System.out.println("Shortest path: " + shortestPath);

            // Calculate the total distance for the shortest path
            int totalDistance = 0;
            for (int i = 0; i < shortestPath.size() - 1; i++) {
                int from = shortestPath.get(i);
                int to = shortestPath.get(i + 1);
                for (Edge edge : graph.get(from)) {
                    if (edge.to == to) {
                        totalDistance += edge.weight;
                        break;
                    }
                }
            }
            System.out.println("Total distance: " + totalDistance);
        }
    }
}