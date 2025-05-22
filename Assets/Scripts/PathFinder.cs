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
    [HideInInspector] public bool hasCompleted;
    [HideInInspector] public PathfindingAlgo pathfindingAlgo;
    GraphController graphController;
    Node startNode = null;
    Node endNode = null;
    public float delayForEachIteration = 0.1f;

    private void Awake()
    {
        graphController = FindObjectOfType<GraphController>();
    }

    public void InitializePathfinder(PathfindingAlgo pathfindingAlgo)
    {
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
        Stack<Node> frontier = new Stack<Node>();
        HashSet<Node> visited = new HashSet<Node>();
        frontier.Push(startNode);
        startNode.parentNode = null;
        while (frontier.Count > 0)
        {
            Node currentNode = frontier.Pop();
            if (currentNode != startNode && currentNode != endNode)
                graphController.ColorNode(currentNode.graphPosition, graphController.visitedTileSprite);
            if (currentNode == endNode)
            {
                Node pathNode = endNode.parentNode;
                while (pathNode != null && pathNode != startNode)
                {
                    graphController.ColorNode(pathNode.graphPosition, graphController.pathTileSprite);
                    pathNode = pathNode.parentNode;
                    yield return new WaitForSeconds(delayForEachIteration);
                }
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
                    if (neighbor != endNode)
                        graphController.ColorNode(neighbor.graphPosition, graphController.frontierTileSprite);
                }
            }
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
    
    public void ResetPathfinding()
    {
        hasCompleted = false;
        graphController.ResetGraph();
    }
}
