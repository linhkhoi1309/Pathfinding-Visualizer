using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class GraphController : MonoBehaviour
{
    public Tilemap tilemap;
    public Sprite startTileSprite;
    public Sprite endTileSprite;
    public Sprite visitedTileSprite;
    public Sprite obstacleTileSprite;
    public Sprite pathTileSprite;
    public Sprite frontierTileSprite;
    public Sprite defaultTileSprite;
    public Vector2Int gridLowerBound;
    public Vector2Int gridUpperBound;

    [HideInInspector] public int graphWidth;
    [HideInInspector] public int graphHeight;
    [HideInInspector] public Node[,] graph;

    public Node startNode = null;
    public Node endNode = null;

    private void Awake()
    {
        InitializeGraph();
    }

    public void InitializeGraph()
    {
        graphWidth = gridUpperBound.x - gridLowerBound.x + 1;
        graphHeight = gridUpperBound.y - gridLowerBound.y + 1;
        graph = new Node[graphWidth, graphHeight];

        for (int x = 0; x < graphWidth; x++)
        {
            for (int y = 0; y < graphHeight; y++)
            {
                graph[x, y] = new Node(new Vector2Int(x, y));
            }
        }
    }

    public bool IsNodeWithinBounds(Vector2Int graphPosition)
    {
        return graphPosition.x >= 0 && graphPosition.x < graphWidth && graphPosition.y >= 0 && graphPosition.y < graphHeight;
    }

    public Node GetNode(Vector2Int graphPosition)
    {
        if (!IsNodeWithinBounds(graphPosition)) return null;
        return graph[graphPosition.x, graphPosition.y];
    }

    public List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();
        Vector2Int[] directions = { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right };

        foreach (var direction in directions)
        {
            Node neighbor = GetNode(node.graphPosition + direction);
            if (neighbor != null) neighbors.Add(neighbor);
        }
        return neighbors;
    }

    public void ColorNodes(List<Node> nodes, Sprite sprite)
    {
        foreach (Node node in nodes)
        {
            ColorNode(node.graphPosition, sprite);
        }
    }

    public void ColorNode(Vector2Int graphPosition, Sprite sprite)
    {
        if (!IsNodeWithinBounds(graphPosition)) return;
        Vector3Int cellPosition = new Vector3Int(graphPosition.x + gridLowerBound.x, graphPosition.y + gridLowerBound.y, 0);
        Tile tile = ScriptableObject.CreateInstance<Tile>();
        tile.sprite = sprite;
        tilemap.SetTile(cellPosition, tile);
    }
    public void SetObstacleNode(Vector2Int graphPosition)
    {
        Node node = GetNode(graphPosition);
        node.isPassable = false;
        ColorNode(graphPosition, obstacleTileSprite);
    }

    public void SetStartNode(Vector2Int graphPosition)
    {
        if (startNode != null) ColorNode(startNode.graphPosition, defaultTileSprite);
        startNode = GetNode(graphPosition);
        ColorNode(graphPosition, startTileSprite);
    }

    public void SetEndNode(Vector2Int graphPosition)
    {
        if (endNode != null) ColorNode(endNode.graphPosition, defaultTileSprite);
        endNode = GetNode(graphPosition);
        ColorNode(graphPosition, endTileSprite);
    }

    public void ClearNode(Vector2Int graphPosition)
    {
        Node node = GetNode(graphPosition);
        node.isPassable = true;
        ColorNode(graphPosition, defaultTileSprite);
        if (node == startNode) startNode = null;
        else if (node == endNode) endNode = null;
    }

    public void ResetGraph()
    {
        for (int x = 0; x < graphWidth; x++)
        {
            for (int y = 0; y < graphHeight; y++)
            {
                ClearNode(new Vector2Int(x, y));
            }
        }
    }
}
