using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum PathfindingAlgo
{
    DFS,
    BFS,
    UCS,
    AStar,
    GreedyBestFirstSearch,
    IDAStar,
    IDDFS,
    BidirectionalSearch
}

[RequireComponent(typeof(LineRenderer))]
public class PathFinder : MonoBehaviour
{
    GraphController graphController;
    LineRenderer lineRenderer;
    Node startNode = null;
    Node endNode = null;
    public float delayForEachIteration = 0f;

    [HideInInspector] public bool hasCompleted = true;
    [HideInInspector] public PathfindingAlgo pathfindingAlgo;
    [HideInInspector] public int numOfNodesExplored = 0;
    [HideInInspector] public float totalCost = 0;
    [HideInInspector] public float processingTime = 0f;

    private void Awake()
    {
        graphController = FindFirstObjectByType<GraphController>();
        lineRenderer = GetComponent<LineRenderer>();
    }

    public void InitializePathfinder(PathfindingAlgo pathfindingAlgo)
    {
        ResetPathFindingConfigs();
        startNode = graphController.startNode;
        endNode = graphController.endNode;
        if (startNode == null || endNode == null)
        {
            Debug.LogError("Start or End node is not set.");
            return;
        }
        this.pathfindingAlgo = pathfindingAlgo;
        hasCompleted = false;
        switch (pathfindingAlgo)
        {
            case PathfindingAlgo.DFS:
                StartCoroutine(DFS());
                break;
            case PathfindingAlgo.BFS:
                StartCoroutine(BFS());
                break;
            case PathfindingAlgo.UCS:
                StartCoroutine(UCS());
                break;
            case PathfindingAlgo.AStar:
                StartCoroutine(AStar());
                break;
            case PathfindingAlgo.GreedyBestFirstSearch:
                StartCoroutine(GreedyBestFirstSearch());
                break;
            case PathfindingAlgo.IDAStar:
                StartCoroutine(IDAStar());
                break;
            case PathfindingAlgo.IDDFS:
                StartCoroutine(IDDFS());
                break;
            case PathfindingAlgo.BidirectionalSearch:
                StartCoroutine(BidirectionalSearch());
                break;
        }
    }

    private IEnumerator DFS()
    {
        float startTime = Time.realtimeSinceStartup;
        yield return null; // Wait for the next frame
        Stack<Node> frontier = new Stack<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        List<Node> path = new List<Node>();
        frontier.Push(startNode);
        startNode.parentNode = null;
        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Pop();
            numOfNodesExplored++;
            // Color the visited nodes
            if (currentNode != startNode && currentNode != endNode)
                graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);
            if (currentNode == endNode)
            {
                Node pathNode = endNode.parentNode;
                path.Add(endNode);
                totalCost += graphController.GetNodeDistance(pathNode, endNode);
                while (pathNode != null && pathNode != startNode)
                {
                    totalCost += graphController.GetNodeDistance(pathNode.parentNode, pathNode);
                    graphController.ColorNode(pathNode.graphPosition, graphController.pathTileSprite);
                    path.Add(pathNode);
                    pathNode = pathNode.parentNode;
                    yield return new WaitForSeconds(delayForEachIteration);
                }
                path.Add(startNode);
                DrawPathLine(path);
                hasCompleted = true;
                yield break;
            }
            visited.Add(currentNode);
            foreach (Node neighbor in graphController.GetNeighbors(currentNode, true))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    frontier.Push(neighbor);
                    neighbor.parentNode = currentNode;
                    // Color the frontier nodes
                    if (neighbor != endNode)
                        graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                }
            }
            processingTime = Time.realtimeSinceStartup - startTime;
            yield return new WaitForSeconds(delayForEachIteration);
        }
        Debug.Log("No path found.");
    }

    private IEnumerator BFS()
    {
        yield return null;
    }

    private IEnumerator UCS()
    {
        float startTime = Time.realtimeSinceStartup;
        yield return null;

        foreach (Node node in graphController.graph)
            node.distanceTraveled = Mathf.Infinity;

        startNode.distanceTraveled = 0;

        PriorityQueue<Node> frontier = new PriorityQueue<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        List<Node> path = new List<Node>();

        frontier.Enqueue(startNode);

        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            numOfNodesExplored++;

            if (currentNode != startNode && currentNode != endNode)
                graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);

            if (currentNode == endNode)
            {
                Node pathNode = endNode.parentNode;
                path.Add(endNode);
                totalCost += graphController.GetNodeDistance(pathNode, endNode);

                while (pathNode != null && pathNode != startNode)
                {
                    totalCost += graphController.GetNodeDistance(pathNode.parentNode, pathNode);
                    graphController.ColorNode(pathNode.graphPosition, graphController.pathTileSprite);
                    path.Add(pathNode);
                    pathNode = pathNode.parentNode;
                    yield return new WaitForSeconds(delayForEachIteration);
                }

                path.Add(startNode);
                DrawPathLine(path);
                hasCompleted = true;
                yield break;
            }

            visited.Add(currentNode);

            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    float distanceToNeighbor = graphController.GetNodeDistance(currentNode, neighbor);
                    float newDistanceTraveled = distanceToNeighbor + currentNode.distanceTraveled;

                    if (float.IsPositiveInfinity(neighbor.distanceTraveled) || newDistanceTraveled < neighbor.distanceTraveled)
                    {
                        neighbor.parentNode = currentNode;
                        neighbor.distanceTraveled = newDistanceTraveled;
                        neighbor.priority = neighbor.distanceTraveled;
                        frontier.Enqueue(neighbor);
                        if (neighbor != endNode)
                            graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                    }
                }
            }

            processingTime = Time.realtimeSinceStartup - startTime;
            yield return new WaitForSeconds(delayForEachIteration);
        }

        Debug.Log("No path found.");
    }

    private IEnumerator AStar()
    {
        float startTime = Time.realtimeSinceStartup;
        yield return null; // Wait for the next frame
        foreach (Node node in graphController.graph)
            node.distanceTraveled = Mathf.Infinity;
        startNode.distanceTraveled = 0;
        PriorityQueue<Node> frontier = new PriorityQueue<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        frontier.Enqueue(startNode);
        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            numOfNodesExplored++;
            if (currentNode != startNode && currentNode != endNode)
                graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);
            if (currentNode == endNode)
            {
                Node pathNode = endNode.parentNode;
                List<Node> path = new List<Node>();
                path.Add(endNode);
                totalCost += graphController.GetNodeDistance(pathNode, endNode);
                while (pathNode != null && pathNode != startNode)
                {
                    totalCost += graphController.GetNodeDistance(pathNode.parentNode, pathNode);
                    graphController.ColorNode(pathNode.graphPosition, graphController.pathTileSprite);
                    path.Add(pathNode);
                    pathNode = pathNode.parentNode;
                    yield return new WaitForSeconds(delayForEachIteration);
                }
                path.Add(startNode);
                DrawPathLine(path);
                hasCompleted = true;
                yield break;
            }
            visited.Add(currentNode);
            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    float distanceToNeighbor = graphController.GetNodeDistance(currentNode, neighbor);
                    float newDistanceTraveled = distanceToNeighbor + currentNode.distanceTraveled;

                    if (float.IsPositiveInfinity(neighbor.distanceTraveled) || newDistanceTraveled < neighbor.distanceTraveled)
                    {
                        neighbor.parentNode = currentNode;
                        neighbor.distanceTraveled = newDistanceTraveled;
                        float distanceToEndNode = graphController.GetNodeDistance(neighbor, endNode);
                        neighbor.priority = neighbor.distanceTraveled + distanceToEndNode;
                        frontier.Enqueue(neighbor);
                        if (neighbor != endNode)
                            graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                    }
                }
            }
            processingTime = Time.realtimeSinceStartup - startTime;
            yield return new WaitForSeconds(delayForEachIteration);
        }
        Debug.Log("No path found.");
    }

    private IEnumerator GreedyBestFirstSearch()
    {
        float startTime = Time.realtimeSinceStartup;
        yield return null; // Wait for the next frame

        PriorityQueue<Node> frontier = new PriorityQueue<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        List<Node> path = new List<Node>();

        // Set priority based on heuristic (distance to end)
        startNode.priority = graphController.GetNodeDistance(startNode, endNode);
        frontier.Enqueue(startNode);

        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            numOfNodesExplored++;

            if (currentNode != startNode && currentNode != endNode)
                graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);

            if (currentNode == endNode)
            {
                Node pathNode = endNode.parentNode;
                path.Add(endNode);
                totalCost += graphController.GetNodeDistance(pathNode, endNode);

                while (pathNode != null && pathNode != startNode)
                {
                    totalCost += graphController.GetNodeDistance(pathNode.parentNode, pathNode);
                    graphController.ColorNode(pathNode.graphPosition, graphController.pathTileSprite);
                    path.Add(pathNode);
                    pathNode = pathNode.parentNode;
                    yield return new WaitForSeconds(delayForEachIteration);
                }

                path.Add(startNode);
                DrawPathLine(path);
                hasCompleted = true;
                processingTime = Time.realtimeSinceStartup - startTime;
                yield break;
            }

            visited.Add(currentNode);

            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    neighbor.parentNode = currentNode;

                    // Heuristic only (Greedy Best-First Search)
                    neighbor.priority = graphController.GetNodeDistance(neighbor, endNode);

                    frontier.Enqueue(neighbor);

                    if (neighbor != endNode)
                        graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                }
            }

            processingTime = Time.realtimeSinceStartup - startTime;
            yield return new WaitForSeconds(delayForEachIteration);
        }

        Debug.Log("No path found.");
        hasCompleted = true;
    }


    private IEnumerator IDAStar()
    {
        yield return null;
    }

    private IEnumerator IDDFS()
    {
        yield return null;
    }

    private IEnumerator BidirectionalSearch()
    {
        float startTime = Time.realtimeSinceStartup;
        yield return null;

        Queue<Node> frontierStart = new Queue<Node>();
        Queue<Node> frontierEnd = new Queue<Node>();
        Dictionary<Node, Node> parentStart = new Dictionary<Node, Node>();
        Dictionary<Node, Node> parentEnd = new Dictionary<Node, Node>();
        HashSet<Node> visitedStart = new HashSet<Node>();
        HashSet<Node> visitedEnd = new HashSet<Node>();

        frontierStart.Enqueue(startNode);
        frontierEnd.Enqueue(endNode);
        parentStart[startNode] = null;
        parentEnd[endNode] = null;

        Node meetingNode = null;

        while (frontierStart.Count > 0 && frontierEnd.Count > 0)
        {
            // Search from start
            Node currentStart = frontierStart.Dequeue();
            visitedStart.Add(currentStart);
            if (currentStart != startNode && currentStart != endNode)
                graphController.ColorNode(currentStart.graphPosition, graphController.visitedTileSprite);
            foreach (Node neighbor in graphController.GetNeighbors(currentStart))
            {
                if (!visitedStart.Contains(neighbor) && neighbor.isPassable)
                {
                    if (!parentStart.ContainsKey(neighbor))
                    {
                        parentStart[neighbor] = currentStart;
                        frontierStart.Enqueue(neighbor);
                        if (neighbor != endNode)
                            graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                    }
                    if (visitedEnd.Contains(neighbor))
                    {
                        meetingNode = neighbor;
                        goto ConstructPath;
                    }
                }
            }

            // Search from end
            Node currentEnd = frontierEnd.Dequeue();
            visitedEnd.Add(currentEnd);
            if (currentEnd != startNode && currentEnd != endNode)
                graphController.ColorNode(currentEnd.graphPosition, graphController.visitedTileSprite);
            foreach (Node neighbor in graphController.GetNeighbors(currentEnd))
            {
                if (!visitedEnd.Contains(neighbor) && neighbor.isPassable)
                {
                    if (!parentEnd.ContainsKey(neighbor))
                    {
                        parentEnd[neighbor] = currentEnd;
                        frontierEnd.Enqueue(neighbor);
                        if (neighbor != startNode)
                            graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                    }
                    if (visitedStart.Contains(neighbor))
                    {
                        meetingNode = neighbor;
                        goto ConstructPath;
                    }
                }
            }

            numOfNodesExplored++;
            processingTime = Time.realtimeSinceStartup - startTime;
            yield return new WaitForSeconds(delayForEachIteration);
        }

        Debug.Log("No path found.");
        hasCompleted = true;
        yield break;

    ConstructPath:
        List<Node> path = new List<Node>();
        Node node = meetingNode;

        while (node != null)
        {
            path.Add(node);
            node = parentStart.ContainsKey(node) ? parentStart[node] : null;
        }
        path.Reverse();

        node = parentEnd[meetingNode];
        while (node != null)
        {
            path.Add(node);
            node = parentEnd[node];
        }

        for (int i = 1; i < path.Count - 1; i++)
        {
            totalCost += graphController.GetNodeDistance(path[i - 1], path[i]);
            graphController.ColorNode(path[i].graphPosition, graphController.pathTileSprite);
            yield return new WaitForSeconds(delayForEachIteration);
        }

        DrawPathLine(path);
        hasCompleted = true;
        processingTime = Time.realtimeSinceStartup - startTime;
    }

    public void ResetPathFinding()
    {
        ResetPathFindingConfigs();
        graphController.ResetGraph();
    }

    public void ResetPathFindingConfigs()
    {
        lineRenderer.positionCount = 0;
        hasCompleted = true;
        numOfNodesExplored = 0;
        totalCost = 0;
        processingTime = 0f;
    }

    private void DrawPathLine(List<Node> path)
    {
        Vector3[] positions = new Vector3[path.Count];
        for (int i = 0; i < path.Count; i++)
        {
            Vector3Int cellPos = (Vector3Int)(graphController.gridLowerBound + path[i].graphPosition);
            Vector3 worldPos = graphController.currentTilemap.GetCellCenterWorld(cellPos);
            positions[i] = worldPos;
        }
        lineRenderer.positionCount = positions.Length;
        lineRenderer.SetPositions(positions);
    }
}
