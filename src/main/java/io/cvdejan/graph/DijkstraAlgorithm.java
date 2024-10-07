package io.cvdejan.graph;

import java.util.*;
import io.cvdejan.graph.RandomDirectedGraphGenerator.Edge;

public class DijkstraAlgorithm {
    // Function to calculate the eccentricity of all vertices
    public static Map<Integer, Integer> calculateEccentricities(Map<Integer, List<Edge>> graph) {
        Map<Integer, Integer> eccentricities = new HashMap<>();

        // For each vertex, compute the eccentricity using Dijkstra's algorithm
        for (int vertex : graph.keySet()) {
            int eccentricity = calculateEccentricity(graph, vertex);
            eccentricities.put(vertex, eccentricity);
        }

        return eccentricities;
    }

    // Helper function to calculate the eccentricity of a single vertex
    private static int calculateEccentricity(Map<Integer, List<Edge>> graph, int vertex) {
        // Use Dijkstra's algorithm to compute the shortest paths from 'vertex' to all other vertices
        Map<Integer, Integer> distances = dijkstraShortestPaths(graph, vertex);

        // The eccentricity is the maximum of these distances
        int eccentricity = distances.values()
                .stream()
                .max(Integer::compare)
                .orElse(Integer.MAX_VALUE);
        return eccentricity;
    }

    // Dijkstra's algorithm to compute the shortest paths from a start vertex to all other vertices
    private static Map<Integer, Integer> dijkstraShortestPaths(Map<Integer, List<Edge>> graph, int start) {
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(node -> node.distance));
        Map<Integer, Integer> distances = new HashMap<>();
        Set<Integer> visited = new HashSet<>();

        // Initialize distances to infinity, except for the start vertex
        for (int vertex : graph.keySet()) {
            distances.put(vertex, Integer.MAX_VALUE);
        }

        distances.put(start, 0);

        pq.add(new Node(start, 0));

        while (!pq.isEmpty()) {
            Node currentNode = pq.poll();
            int currentVertex = currentNode.vertex;

            // If this vertex was already visited, skip it
            if (visited.contains(currentVertex)) {
                continue;
            }
            visited.add(currentVertex);

            // Process each neighbor of the current vertex
            for (Edge edge : graph.get(currentVertex)) {
                int neighbor = edge.to;
                int newDist = distances.get(currentVertex) + edge.weight;

                // If a shorter path to the neighbor is found, update the distance and add it to the priority queue
                if (newDist < distances.get(neighbor)) {
                    distances.put(neighbor, newDist);
                    pq.add(new Node(neighbor, newDist));
                }
            }
        }

        return distances;
    }

    // Calculate the radius and diameter of the graph
    public static void calculateRadiusAndDiameter(Map<Integer, List<Edge>> graph) {
        Map<Integer, Integer> eccentricities = calculateEccentricities(graph);

        // The radius is the minimum eccentricity
        int radius = eccentricities.values()
                .stream()
                .min(Integer::compare)
                .orElse(Integer.MAX_VALUE);

        // The diameter is the maximum eccentricity
        int diameter = eccentricities.values()
                .stream()
                .max(Integer::compare)
                .orElse(Integer.MIN_VALUE);

        // Output the results
        System.out.println("Eccentricities: " + eccentricities);
        System.out.println("Radius of the graph: " + radius);
        System.out.println("Diameter of the graph: " + diameter);
    }

    // Dijkstra's algorithm to find the shortest path between two vertices
    public static List<Integer> dijkstraShortestPath(Map<Integer, List<Edge>> graph, int start, int end) {
        // Priority queue to pick the next vertex with the smallest known distance
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(node -> node.distance));
        Map<Integer, Integer> distances = new HashMap<>();  // Store shortest known distances to each vertex
        Map<Integer, Integer> previous = new HashMap<>();   // Track the previous vertex to reconstruct the path
        Set<Integer> visited = new HashSet<>();             // Track visited vertices

        // Initialize distances to infinity, except for the start vertex
        for (int vertex : graph.keySet()) {
            distances.put(vertex, Integer.MAX_VALUE);
        }
        distances.put(start, 0);

        pq.add(new Node(start, 0));

        while (!pq.isEmpty()) {
            Node currentNode = pq.poll();
            int currentVertex = currentNode.vertex;

            // If the current vertex is the target, we can stop the search
            if (currentVertex == end) break;

            // If this vertex was already visited, skip it
            if (visited.contains(currentVertex)) {
                continue;
            }
            visited.add(currentVertex);

            // Process each neighbor of the current vertex
            for (Edge edge : graph.get(currentVertex)) {
                int neighbor = edge.to;
                int newDist = distances.get(currentVertex) + edge.weight;

                // If a shorter path to the neighbor is found, update the distance and add it to the priority queue
                if (newDist < distances.get(neighbor)) {
                    distances.put(neighbor, newDist);
                    previous.put(neighbor, currentVertex);
                    pq.add(new Node(neighbor, newDist));
                }
            }
        }

        // Reconstruct the shortest path
        List<Integer> path = new ArrayList<>();
        Integer step = end;

        // If there's no path to the end, return an empty list
        if (!previous.containsKey(end) && start != end) {
            return path;  // No path found
        }

        while (step != null) {
            path.add(step);
            step = previous.get(step);
        }

        // Reverse the path, because we reconstruct it backwards
        Collections.reverse(path);
        return path;
    }

    // Node class to represent vertices in the priority queue
    private static class Node {
        int vertex;
        int distance;

        Node(int vertex, int distance) {
            this.vertex = vertex;
            this.distance = distance;
        }
    }
}
