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
    [HideInInspector] public long memoryUsage = 0;

    private UIController uiController;

    private void Awake()
    {
        graphController = FindFirstObjectByType<GraphController>();
        lineRenderer = GetComponent<LineRenderer>();
    }

    private void Start()
    {
        uiController = FindFirstObjectByType<UIController>();
    }

    public void InitializePathfinder(PathfindingAlgo pathfindingAlgo)
    {
        ResetPathFindingConfigs();
        startNode = graphController.startNode;
        endNode = graphController.endNode;
        if (startNode == null || endNode == null)
        {
            uiController.ShowPopup("Pathfinding Error", "Start node or end node is not set");
            return;
        }
        this.pathfindingAlgo = pathfindingAlgo;
        hasCompleted = false;
        switch (pathfindingAlgo)
        {
            case PathfindingAlgo.DFS:
                StartCoroutine(m_DFS());
                StartCoroutine(DFS());
                break;
            case PathfindingAlgo.BFS:
                StartCoroutine(m_BFS());
                StartCoroutine(BFS());
                break;
            case PathfindingAlgo.UCS:
                StartCoroutine(m_UCS());
                StartCoroutine(UCS());
                break;
            case PathfindingAlgo.AStar:
                StartCoroutine(m_AStar());
                StartCoroutine(AStar());
                break;
            case PathfindingAlgo.GreedyBestFirstSearch:
                StartCoroutine(m_GreedyBestFirstSearch());
                StartCoroutine(GreedyBestFirstSearch());
                break;
            case PathfindingAlgo.IDAStar:
                StartCoroutine(m_IDAStar());
                StartCoroutine(IDAStar());
                break;
            case PathfindingAlgo.IDDFS:
                StartCoroutine(m_IDDFS());
                StartCoroutine(IDDFS());
                break;
            case PathfindingAlgo.BidirectionalSearch:
                StartCoroutine(m_BidirectionalSearch());
                StartCoroutine(BidirectionalSearch());
                break;
        }
    }

    private IEnumerator DFS()
    {
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
            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
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
            yield return new WaitForSeconds(delayForEachIteration);
        }
        uiController.ShowPopup("Pathfinding Error", "No path found");
    }

    private IEnumerator m_DFS()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        Stack<Node> frontier = new Stack<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        frontier.Push(startNode);
        startNode.parentNode = null;
        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Pop();
            if (currentNode == endNode)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
                yield break;
            }
            visited.Add(currentNode);
            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    frontier.Push(neighbor);
                    neighbor.parentNode = currentNode;
                }
            }
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
        yield return null;
    }

    private IEnumerator BFS()
    {
        Queue<Node> frontier = new Queue<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        List<Node> path = new List<Node>();

        frontier.Enqueue(startNode);
        visited.Add(startNode);
        startNode.parentNode = null;

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

            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    neighbor.parentNode = currentNode;
                    frontier.Enqueue(neighbor);
                    visited.Add(neighbor);
                    if (neighbor != endNode)
                        graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                }
            }
            yield return new WaitForSeconds(delayForEachIteration);
        }

        uiController.ShowPopup("Pathfinding Error", "No path found");
    }

    private IEnumerator m_BFS()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        Queue<Node> frontier = new Queue<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        frontier.Enqueue(startNode);
        visited.Add(startNode);
        startNode.parentNode = null;
        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            if (currentNode == endNode)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
                yield break;
            }
            foreach (Node neighbor in graphController.GetNeighbors(currentNode))
            {
                if (!visited.Contains(neighbor) && neighbor.isPassable)
                {
                    neighbor.parentNode = currentNode;
                    frontier.Enqueue(neighbor);
                    visited.Add(neighbor);
                }
            }
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
        yield return null;
    }

    private IEnumerator UCS()
    {
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
            yield return new WaitForSeconds(delayForEachIteration);
        }

        uiController.ShowPopup("Pathfinding Error", "No path found");
    }

    private IEnumerator m_UCS()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        foreach (Node node in graphController.graph)
            node.distanceTraveled = Mathf.Infinity;

        startNode.distanceTraveled = 0;

        PriorityQueue<Node> frontier = new PriorityQueue<Node>();
        HashSet<Node> visited = new HashSet<Node>();

        frontier.Enqueue(startNode);

        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            if (currentNode == endNode)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
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
                    }
                }
            }
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
    }

    private IEnumerator AStar()
    {
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
            yield return new WaitForSeconds(delayForEachIteration);
        }
        uiController.ShowPopup("Pathfinding Error", "No path found");
    }

    private IEnumerator m_AStar()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        foreach (Node node in graphController.graph)
            node.distanceTraveled = Mathf.Infinity;
        startNode.distanceTraveled = 0;
        PriorityQueue<Node> frontier = new PriorityQueue<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        frontier.Enqueue(startNode);
        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            if (currentNode == endNode)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
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
                    }
                }
            }
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
    }

    private IEnumerator GreedyBestFirstSearch()
    {
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
            yield return new WaitForSeconds(delayForEachIteration);
        }

        uiController.ShowPopup("Pathfinding Error", "No path found");
        hasCompleted = true;
    }

    private IEnumerator m_GreedyBestFirstSearch()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        PriorityQueue<Node> frontier = new PriorityQueue<Node>();
        HashSet<Node> visited = new HashSet<Node>();

        // Set priority based on heuristic (distance to end)
        startNode.priority = graphController.GetNodeDistance(startNode, endNode);
        frontier.Enqueue(startNode);

        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Dequeue();
            if (currentNode == endNode)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
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
                }
            }
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
        yield return null;
    }


    private IEnumerator IDAStar()
    {
        float heuristic(Node n) => graphController.GetNodeDistance(n, endNode);

        foreach (Node node in graphController.graph)
        {
            node.distanceTraveled = Mathf.Infinity;
            node.parentNode = null;
        }

        startNode.distanceTraveled = 0;
        float threshold = heuristic(startNode);

        while (true)
        {
            foreach (Node node in graphController.graph)
            {
                node.distanceTraveled = Mathf.Infinity;
                node.parentNode = null;
            }

            startNode.distanceTraveled = 0;

            float minThreshold = Mathf.Infinity;
            Stack<Node> frontier = new Stack<Node>();
            //HashSet<Node> visited = new HashSet<Node>();
            frontier.Push(startNode);
            //visited.Add(startNode);

            while (frontier.Count > 0)
            {
                Node currentNode = frontier.Pop();
                numOfNodesExplored++;

                float f = currentNode.distanceTraveled + heuristic(currentNode);
                if (f > threshold + 0.0001f)
                {
                    minThreshold = Mathf.Min(minThreshold, f);
                    if (currentNode != endNode)
                        graphController.ColorNode(currentNode.graphPosition, graphController.frontierTileSprite);
                    continue;
                }

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

                foreach (Node neighbor in graphController.GetNeighbors(currentNode))
                {
                    if (!neighbor.isPassable) continue;

                    float tentativeG = currentNode.distanceTraveled + graphController.GetNodeDistance(currentNode, neighbor);

                    if (tentativeG < neighbor.distanceTraveled - 0.0001f)
                    {
                        neighbor.distanceTraveled = tentativeG;
                        neighbor.parentNode = currentNode;
                        frontier.Push(neighbor);
                        //visited.Add(neighbor);
                    }
                }

                yield return new WaitForSeconds(delayForEachIteration);
            }

            if (minThreshold == Mathf.Infinity)
            {
                uiController.ShowPopup("Pathfinding Error", "No path found");
                hasCompleted = true;
                yield break;
            }

            threshold = minThreshold;
        }
    }

    private IEnumerator m_IDAStar()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        float heuristic(Node n) => graphController.GetNodeDistance(n, endNode);

        foreach (Node node in graphController.graph)
        {
            node.distanceTraveled = Mathf.Infinity;
            node.parentNode = null;
        }

        startNode.distanceTraveled = 0;
        float threshold = heuristic(startNode);

        while (true)
        {
            foreach (Node node in graphController.graph)
            {
                node.distanceTraveled = Mathf.Infinity;
                node.parentNode = null;
            }

            startNode.distanceTraveled = 0;

            float minThreshold = Mathf.Infinity;
            Stack<Node> frontier = new Stack<Node>();
            frontier.Push(startNode);

            while (frontier.Count > 0)
            {
                Node currentNode = frontier.Pop();
                if (currentNode == endNode)
                {
                    processingTime = Time.realtimeSinceStartup - startTime;
                    memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
                    yield break;
                }

                float f = currentNode.distanceTraveled + heuristic(currentNode);
                if (f > threshold + 0.0001f)
                {
                    minThreshold = Mathf.Min(minThreshold, f);
                    continue;
                }

                foreach (Node neighbor in graphController.GetNeighbors(currentNode))
                {
                    if (!neighbor.isPassable) continue;

                    float tentativeG = currentNode.distanceTraveled + graphController.GetNodeDistance(currentNode, neighbor);

                    if (tentativeG < neighbor.distanceTraveled - 0.0001f)
                    {
                        neighbor.distanceTraveled = tentativeG;
                        neighbor.parentNode = currentNode;
                        frontier.Push(neighbor);
                    }
                }
            }

            if (minThreshold == Mathf.Infinity)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
                yield break;
            }
            threshold = minThreshold;
        }
    }

    private IEnumerator IDDFS()
    {
        int depth = 0;
        bool pathFound = false;
        bool allReachableNodesVisited = false;

        int maxDepth = graphController.graph.Length;

        while (!pathFound && !allReachableNodesVisited && depth <= maxDepth)
        {
            foreach (Node node in graphController.graph)
            {
                node.parentNode = null;
            }

            HashSet<Node> visited = new HashSet<Node>();
            pathFound = DLS(startNode, endNode, depth, visited);

            allReachableNodesVisited = true;
            foreach (Node node in graphController.graph)
            {
                if (node.isPassable && !visited.Contains(node))
                {
                    allReachableNodesVisited = false;
                    break;
                }
            }

            if (pathFound)
            {
                List<Node> path = new List<Node>();
                Node pathNode = endNode;
                totalCost = 0;

                while (pathNode != null)
                {
                    path.Add(pathNode);
                    if (pathNode.parentNode != null)
                        totalCost += graphController.GetNodeDistance(pathNode.parentNode, pathNode);
                    pathNode = pathNode.parentNode;
                }

                path.Reverse();

                foreach (Node node in path)
                {
                    if (node != startNode && node != endNode)
                    {
                        graphController.ColorNode(node.graphPosition, graphController.pathTileSprite);
                        yield return new WaitForSeconds(delayForEachIteration);
                    }
                }

                DrawPathLine(path);
                hasCompleted = true;
                yield break;
            }

            depth++;
            yield return new WaitForSeconds(delayForEachIteration);
        }

        uiController.ShowPopup("Pathfinding Error", "No path found");
        hasCompleted = true;
    }

    private bool DLS(Node currentNode, Node targetNode, int depthLimit, HashSet<Node> visited)
    {
        numOfNodesExplored++;
        if (currentNode == targetNode)
            return true;

        if (depthLimit == 0)
            return false;

        visited.Add(currentNode);

        foreach (Node neighbor in graphController.GetNeighbors(currentNode))
        {
            if (!neighbor.isPassable || visited.Contains(neighbor))
                continue;

            neighbor.parentNode = currentNode;

            if (neighbor != targetNode)
                graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);

            bool found = DLS(neighbor, targetNode, depthLimit - 1, visited);
            if (found)
                return true;
        }

        if (currentNode != startNode && currentNode != targetNode)
            graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);

        return false;
    }

    private bool m_DLS(Node currentNode, Node targetNode, int depthLimit, HashSet<Node> visited)
    {
        if (currentNode == targetNode)
            return true;
        if (depthLimit == 0)
            return false;
        visited.Add(currentNode);
        foreach (Node neighbor in graphController.GetNeighbors(currentNode))
        {
            if (!neighbor.isPassable || visited.Contains(neighbor)) continue;
            neighbor.parentNode = currentNode;
            bool found = m_DLS(neighbor, targetNode, depthLimit - 1, visited);
            if (found)
                return true;
        }
        return false;
    }


    private IEnumerator m_IDDFS()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
        int depth = 0;
        bool pathFound = false;
        bool allReachableNodesVisited = false;

        int maxDepth = graphController.graph.Length;

        while (!pathFound && !allReachableNodesVisited && depth <= maxDepth)
        {
            foreach (Node node in graphController.graph)
            {
                node.parentNode = null;
            }

            HashSet<Node> visited = new HashSet<Node>();
            pathFound = m_DLS(startNode, endNode, depth, visited);

            allReachableNodesVisited = true;
            foreach (Node node in graphController.graph)
            {
                if (node.isPassable && !visited.Contains(node))
                {
                    allReachableNodesVisited = false;
                    break;
                }
            }

            if (pathFound)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
                yield break;
            }
            depth++;
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
    }

    private IEnumerator BidirectionalSearch()
    {
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
            yield return new WaitForSeconds(delayForEachIteration);
        }

        uiController.ShowPopup("Pathfinding Error", "No path found");
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

        for (int i = 1; i < path.Count; i++)
        {
            totalCost += graphController.GetNodeDistance(path[i - 1], path[i]);
            if (i < path.Count - 1)
                graphController.ColorNode(path[i].graphPosition, graphController.pathTileSprite);
            yield return new WaitForSeconds(delayForEachIteration);
        }

        DrawPathLine(path);
        hasCompleted = true;
    }

    private IEnumerator m_BidirectionalSearch()
    {
        long memBefore = System.GC.GetTotalMemory(true);
        float startTime = Time.realtimeSinceStartup;
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
            foreach (Node neighbor in graphController.GetNeighbors(currentStart))
            {
                if (!visitedStart.Contains(neighbor) && neighbor.isPassable)
                {
                    if (!parentStart.ContainsKey(neighbor))
                    {
                        parentStart[neighbor] = currentStart;
                        frontierStart.Enqueue(neighbor);
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
            foreach (Node neighbor in graphController.GetNeighbors(currentEnd))
            {
                if (!visitedEnd.Contains(neighbor) && neighbor.isPassable)
                {
                    if (!parentEnd.ContainsKey(neighbor))
                    {
                        parentEnd[neighbor] = currentEnd;
                        frontierEnd.Enqueue(neighbor);
                    }
                    if (visitedStart.Contains(neighbor))
                    {
                        meetingNode = neighbor;
                        goto ConstructPath;
                    }
                }
            }
        }
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
        yield break;

    ConstructPath:
        processingTime = Time.realtimeSinceStartup - startTime;
        memoryUsage = System.GC.GetTotalMemory(false) - memBefore;
        yield return null;
    }
    public void ResetPathFinding()
    {
        ResetPathFindingConfigs();
        StopAllCoroutines();
        graphController.ResetGraph();
    }

    public void ResetPathFindingConfigs()
    {
        lineRenderer.positionCount = 0;
        hasCompleted = true;
        numOfNodesExplored = 0;
        totalCost = 0;
        processingTime = 0f;
        memoryUsage = 0;
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
