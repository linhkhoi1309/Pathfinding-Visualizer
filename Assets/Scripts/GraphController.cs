using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class GraphController : MonoBehaviour
{
    [HideInInspector] public Tilemap currentTilemap = null;
    public Tilemap blankTilemap;
    public Tilemap sampleTilemap;
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
        currentTilemap = blankTilemap;
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

        // Initialize the graph with the prebuilt tilemap
        if (currentTilemap != blankTilemap)
        {
            for (int x = gridLowerBound.x; x <= gridUpperBound.x; x++)
            {
                for (int y = gridLowerBound.y; y <= gridUpperBound.y; y++)
                {
                    Vector3Int cellPosition = new Vector3Int(x, y, 0);
                    Tile tile = currentTilemap.GetTile(cellPosition) as Tile;
                    if (tile != null)
                    {
                        if (tile.sprite == obstacleTileSprite)
                        {
                            SetObstacleNode(new Vector2Int(x - gridLowerBound.x, y - gridLowerBound.y));
                        }
                        else if (tile.sprite == startTileSprite)
                        {
                            SetStartNode(new Vector2Int(x - gridLowerBound.x, y - gridLowerBound.y));
                        }
                        else if (tile.sprite == endTileSprite)
                        {
                            SetEndNode(new Vector2Int(x - gridLowerBound.x, y - gridLowerBound.y));
                        }
                    }
                }
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

    public List<Node> GetNeighbors(Node node, bool isRandomDirections = false)
    {
        List<Node> neighbors = new List<Node>();
        Vector2Int[] directions = { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right,
                                     new Vector2Int(1, 1), new Vector2Int(-1, -1), new Vector2Int(1, -1), new Vector2Int(-1, 1) };

        if (isRandomDirections) Ultility.RandomizeArray(directions);
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
        currentTilemap.SetTile(cellPosition, tile);
    }
    public void SetObstacleNode(Vector2Int graphPosition)
    {
        Node node = GetNode(graphPosition);
        if (node == null) return;
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

    public float GetNodeDistance(Node source, Node target)
    {
        int dx = Mathf.Abs(source.graphPosition.x - target.graphPosition.x);
        int dy = Mathf.Abs(source.graphPosition.y - target.graphPosition.y);

        int min = Mathf.Min(dx, dy);
        int max = Mathf.Max(dx, dy);

        int straightSteps = max - min;
        return 1.4f * min + straightSteps;
    }

    // This method is used to calculate the cost of moving from one node to another surrounding node
    public float GetStepCost(Node from, Node to)
    {
        if (from == null || to == null) return 0f;
        int dx = Mathf.Abs(from.graphPosition.x - to.graphPosition.x);
        int dy = Mathf.Abs(from.graphPosition.y - to.graphPosition.y);
        if (dx == 1 && dy == 1)
            return 1.4f;
        else if ((dx == 1 && dy == 0) || (dx == 0 && dy == 1))
            return 1f;
        else
            return 0f;
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
