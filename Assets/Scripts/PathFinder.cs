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
    [HideInInspector] public int memoryUsage = 0;
    [HideInInspector] public int maxNumInFrontier = 0; 
    [HideInInspector] public bool showSearchTree;
    public GameObject searchTree;
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
                    if (showSearchTree) DrawLine(currentNode, neighbor);
                    frontier.Push(neighbor);
                    neighbor.parentNode = currentNode;
                    memoryUsage = Mathf.Max(memoryUsage, frontier.Count);
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
                    if (showSearchTree) DrawLine(currentNode, neighbor);
                    neighbor.parentNode = currentNode;
                    frontier.Enqueue(neighbor);
                    memoryUsage = Mathf.Max(memoryUsage, frontier.Count);
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
                        if (showSearchTree) DrawLine(currentNode, neighbor);
                        neighbor.parentNode = currentNode;
                        neighbor.distanceTraveled = newDistanceTraveled;
                        neighbor.priority = neighbor.distanceTraveled;
                        frontier.Enqueue(neighbor);
                        memoryUsage = Mathf.Max(memoryUsage, frontier.Count);
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
                        if (showSearchTree) DrawLine(currentNode, neighbor);
                        frontier.Enqueue(neighbor);
                        memoryUsage = Mathf.Max(memoryUsage, frontier.Count);
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
                    if (showSearchTree) DrawLine(currentNode, neighbor);
                    neighbor.parentNode = currentNode;

                    // Heuristic only (Greedy Best-First Search)
                    neighbor.priority = graphController.GetNodeDistance(neighbor, endNode);

                    frontier.Enqueue(neighbor);
                    memoryUsage = Mathf.Max(memoryUsage, frontier.Count);

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
                        memoryUsage = Mathf.Max(memoryUsage, frontier.Count);
                        if (neighbor != endNode)
                            graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                        if (showSearchTree) DrawLine(currentNode, neighbor);
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
                yield break;
            }
            threshold = minThreshold;
        }
    }

    private IEnumerator IDDFS()
    {
        int depthLimit = 0;
        bool found = false;
        HashSet<Node> visited = new HashSet<Node>();
        Stack<Node> frontier = new Stack<Node>();

        while (!found)
        {
            // Reset visited and parent nodes for each iteration
            List<Node> path = new List<Node>();
            visited.Clear();
            frontier.Clear();
            frontier.Push(startNode);
            startNode.parentNode = null;
            visited.Add(startNode);

            // Perform depth-limited DFS
            while (frontier.Count > 0 && !found)
            {
                Node currentNode = frontier.Pop();
                numOfNodesExplored++;

                // Color the visited nodes
                if (currentNode != startNode && currentNode != endNode)
                    graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);

                if (currentNode == endNode)
                {
                    found = true;
                    // Reconstruct path
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

                // Only explore neighbors if we haven't reached depth limit
                if (GetNodeDepth(currentNode) < depthLimit)
                {
                    foreach (Node neighbor in graphController.GetNeighbors(currentNode))
                    {
                        if (!visited.Contains(neighbor) && neighbor.isPassable)
                        {
                            frontier.Push(neighbor);
                            memoryUsage = Mathf.Max(memoryUsage, frontier.Count);
                            neighbor.parentNode = currentNode;
                            visited.Add(neighbor);
                            if (showSearchTree) DrawLine(currentNode, neighbor);
                            // Color the frontier nodes
                            if (neighbor != endNode)
                                graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                        }
                    }
                }

                yield return new WaitForSeconds(delayForEachIteration);
            }

            depthLimit++;

            // Safety check to prevent infinite loop
            if (depthLimit > 62)
            {
                uiController.ShowPopup("Pathfinding Error", "No path found within maximum depth");
                yield break;
            }
        }
    }
    private IEnumerator m_IDDFS()
    {
        float startTime = Time.realtimeSinceStartup;
        int depthLimit = 0;
        bool found = false;
        HashSet<Node> visited = new HashSet<Node>();
        Stack<Node> frontier = new Stack<Node>();

        while (!found)
        {
            visited.Clear();
            frontier.Clear();
            foreach (Node node in graphController.graph)
            {
                node.parentNode = null;
            }

            frontier.Push(startNode);
            startNode.parentNode = null;
            visited.Add(startNode);
            while (frontier.Count > 0 && !found)
            {
                Node currentNode = frontier.Pop();

                if (currentNode == endNode)
                {
                    found = true;
                    processingTime = Time.realtimeSinceStartup - startTime;
                    yield break;
                }

                if (GetNodeDepth(currentNode) < depthLimit)
                {
                    foreach (Node neighbor in graphController.GetNeighbors(currentNode))
                    {
                        if (!visited.Contains(neighbor) && neighbor.isPassable)
                        {
                            frontier.Push(neighbor);
                            neighbor.parentNode = currentNode;
                            visited.Add(neighbor);
                        }
                    }
                }
            }
            depthLimit++;
            if (depthLimit > 62)
            {
                processingTime = Time.realtimeSinceStartup - startTime;
                yield break;
            }

        }
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
                        memoryUsage = Mathf.Max(memoryUsage, frontierStart.Count);
                        if (showSearchTree) DrawLine(currentStart, neighbor);
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
                        memoryUsage = Mathf.Max(memoryUsage, frontierEnd.Count);
                        if (showSearchTree) DrawLine(currentEnd, neighbor);
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

            numOfNodesExplored += 2;
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
        yield break;

    ConstructPath:
        processingTime = Time.realtimeSinceStartup - startTime;
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
        foreach (Transform child in searchTree.transform) Destroy(child.gameObject);
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

    private void DrawLine(Node startNode, Node endNode)
    {
        GameObject branchObj = new GameObject("Branch");
        branchObj.transform.SetParent(searchTree.transform);
        LineRenderer branchRenderer = branchObj.AddComponent<LineRenderer>();
        branchRenderer.material = new Material(Shader.Find("Sprites/Default"));
        branchRenderer.startWidth = 0.1f;
        branchRenderer.sortingOrder = 10;
        Vector3 startWorldPos = graphController.currentTilemap.GetCellCenterWorld((Vector3Int)(graphController.gridLowerBound + startNode.graphPosition));
        Vector3 endWorldPos = graphController.currentTilemap.GetCellCenterWorld((Vector3Int)(graphController.gridLowerBound + endNode.graphPosition));
        branchRenderer.positionCount = 2;
        branchRenderer.SetPosition(0, startWorldPos);
        branchRenderer.SetPosition(1, endWorldPos);

        float arrowSize = 0.2f; // Adjust the size of the arrow head
        Vector2 direction = endWorldPos - startWorldPos;
        Vector2 midWorldPos = (startWorldPos + endWorldPos) / 2;
        Vector2 arrowPoint1 = midWorldPos - new Vector2(-direction.y, direction.x).normalized * arrowSize;
        Vector2 arrowPoint2 = midWorldPos + new Vector2(-direction.y, direction.x).normalized * arrowSize;

        GameObject arrowHead = new GameObject("ArrowHead");
        arrowHead.transform.SetParent(branchObj.transform);
        LineRenderer arrowRenderer = arrowHead.AddComponent<LineRenderer>();
        arrowRenderer.material = new Material(Shader.Find("Sprites/Default"));
        arrowRenderer.startWidth = 0.1f;
        arrowRenderer.sortingOrder = 10;
        arrowRenderer.positionCount = 3;
        arrowRenderer.SetPosition(0, arrowPoint1);
        arrowRenderer.SetPosition(1, endWorldPos);
        arrowRenderer.SetPosition(2, arrowPoint2);  
    }

    private int GetNodeDepth(Node node)
    {
        int depth = 0;
        while (node.parentNode != null)
        {
            depth++;
            node = node.parentNode;
        }
        return depth;
    }
}
