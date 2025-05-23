using System.Collections;
using System.Collections.Generic;
using System.IO;
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
        graphController = FindObjectOfType<GraphController>();
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
            if (currentNode != startNode && currentNode != endNode)
                graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);
            if (currentNode == endNode)
            {
                Node pathNode = endNode.parentNode;
                path.Add(endNode);
                totalCost += graphController.GetStepCost(pathNode, endNode);
                while (pathNode != null && pathNode != startNode)
                {
                    totalCost += graphController.GetStepCost(pathNode.parentNode, pathNode);
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
                    if (neighbor != endNode)
                        graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                }
            }
            processingTime = Time.realtimeSinceStartup - startTime;
            yield return new WaitForSeconds(delayForEachIteration);
        }
    }

    private IEnumerator BFS()
    {
        yield return null;
    }

    private IEnumerator UCS()
    {
        yield return null;
    }

    private IEnumerator AStar()
    {
        yield return null;
    }

    private IEnumerator GreedyBestFirstSearch()
    {
        yield return null;
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
        yield return null;
    }

    public void ResetPathFinding()
    {
        ResetPathFindingConfigs();
        graphController.ResetGraph();
    }

    private void ResetPathFindingConfigs()
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
            Vector3 worldPos = graphController.tilemap.GetCellCenterWorld(cellPos);
            positions[i] = worldPos;
        }
        lineRenderer.positionCount = positions.Length;
        lineRenderer.SetPositions(positions);
    }
}
