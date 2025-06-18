using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class GraphController : MonoBehaviour
{
    [HideInInspector] public Tilemap currentTilemap = null;

    [Header("Tilemaps")]
    public List<Tilemap> tilemaps = new List<Tilemap>();

    [Header("Tiles")]
    public Sprite startTileSprite;
    public Sprite endTileSprite;
    public Sprite visitedTileSprite;
    public Sprite obstacleTileSprite;
    public Sprite pathTileSprite;
    public Sprite frontierTileSprite;
    public Sprite defaultTileSprite;

    [Header("Grid Bounds")]
    public Vector2Int gridLowerBound;
    public Vector2Int gridUpperBound;

    [HideInInspector] public int graphWidth;
    [HideInInspector] public int graphHeight;
    [HideInInspector] public Node[,] graph;

    public Node startNode = null;
    public Node endNode = null;

    public string currentTilemapTag = GlobalConfigs.MAZE_1_TILEMAP;

    private void Awake()
    {
        LoadTilemap(currentTilemapTag);
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

        // Initializes the graph with the prebuilt tilemap
        if (currentTilemapTag != GlobalConfigs.CUSTOM_TILEMAP) // If the current tilemap is not the custom one
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
                            SetObstacleNode(new Vector2Int(x - gridLowerBound.x, y - gridLowerBound.y), true);
                        }
                        else if (tile.sprite == startTileSprite)
                        {
                            SetStartNode(new Vector2Int(x - gridLowerBound.x, y - gridLowerBound.y), true);
                        }
                        else if (tile.sprite == endTileSprite)
                        {
                            SetEndNode(new Vector2Int(x - gridLowerBound.x, y - gridLowerBound.y), true);
                        }
                    }
                }
            }
        } else ResetGraph(); // If the current tilemap is the custom one, reset the graph to default state
    }

    public bool IsNodeWithinBounds(Vector2Int graphPosition)
    {
        return graphPosition.x >= 0 && graphPosition.x < graphWidth && graphPosition.y >= 0 && graphPosition.y < graphHeight;
    }

    // Returns the node at the specified graph position, or null if out of bounds
    public Node GetNode(Vector2Int graphPosition)
    {
        if (!IsNodeWithinBounds(graphPosition)) return null;
        return graph[graphPosition.x, graphPosition.y];
    }

    // Returns a list of neighboring nodes for the given node
    public List<Node> GetNeighbors(Node node)
    {
        return node.neighbors;
    }

    // Colors all nodes in the list with the specified sprite
    public void ColorNodes(List<Node> nodes, Sprite sprite)
    {
        foreach (Node node in nodes)
        {
            ColorNode(node.graphPosition, sprite);
        }
    }

    // Colors a single node at the specified graph position with the given sprite
    public void ColorNode(Vector2Int graphPosition, Sprite sprite)
    {
        if (!IsNodeWithinBounds(graphPosition)) return;
        Vector3Int cellPosition = new Vector3Int(graphPosition.x + gridLowerBound.x, graphPosition.y + gridLowerBound.y, 0);
        Tile tile = ScriptableObject.CreateInstance<Tile>();
        tile.sprite = sprite;
        currentTilemap.SetTile(cellPosition, tile);
    }

    // Sets the node at the specified graph position as an obstacle and colors it with the obstacle tile sprite
    public void SetObstacleNode(Vector2Int graphPosition, bool preserved = false)
    {
        Node node = GetNode(graphPosition);
        if (node == null) return;
        node.isPassable = false;
        node.preserved = preserved;
        ColorNode(graphPosition, obstacleTileSprite);
    }

    // Sets the node at the specified graph position as a start node and colors it with the start tile sprite
    public void SetStartNode(Vector2Int graphPosition, bool preserved = false)
    {
        if (startNode != null) ColorNode(startNode.graphPosition, defaultTileSprite);
        startNode = GetNode(graphPosition);
        startNode.preserved = preserved;
        ColorNode(graphPosition, startTileSprite);
    }

    // Sets the node at the specified graph position as the end node and colors it with the end tile sprite
    public void SetEndNode(Vector2Int graphPosition, bool preserved = false)
    {
        if (endNode != null) ColorNode(endNode.graphPosition, defaultTileSprite);
        endNode = GetNode(graphPosition);
        endNode.preserved = preserved;
        ColorNode(graphPosition, endTileSprite);
    }

    // Clear node at the specified graph position, making it passable and resetting its color
    public void ClearNode(Vector2Int graphPosition)
    {
        Node node = GetNode(graphPosition);
        if (node == null || node.preserved) return;
        node.isPassable = true;
        ColorNode(graphPosition, defaultTileSprite);
    }

    // Returns the optimal distance between two nodes
    public float GetNodeDistance(Node source, Node target)
    {
        int dx = Mathf.Abs(source.graphPosition.x - target.graphPosition.x);
        int dy = Mathf.Abs(source.graphPosition.y - target.graphPosition.y);

        int min = Mathf.Min(dx, dy);
        int max = Mathf.Max(dx, dy);

        int straightSteps = max - min;
        return 1.4f * min + straightSteps;
    }

    // Resets the entire graph, clearing all nodes to default and making them passable
    public void ResetGraph()
    {
        if (startNode != null && !startNode.preserved) startNode = null;
        if (endNode != null && !endNode.preserved) endNode = null;
        for (int x = 0; x < graphWidth; x++)
        {
            for (int y = 0; y < graphHeight; y++)
            {
                ClearNode(new Vector2Int(x, y));
            }
        }
    }

    public void LoadTilemap(string tilemapTag)
    {
        foreach (Tilemap tm in tilemaps)
        {
            if (tm.gameObject.tag == tilemapTag)
            {
                currentTilemap = tm;
                tm.gameObject.SetActive(true);
                continue;
            }
            tm.gameObject.SetActive(false);
        }
        if (currentTilemap == null)
        {
            Debug.LogError("No tilemap found with tag: " + tilemapTag);
            return;
        }
        if (currentTilemapTag != tilemapTag)
        {
            startNode = null;
            endNode = null;
        }
        if (startNode != null && !startNode.preserved) startNode = null;
        if (endNode != null && !endNode.preserved) endNode = null;
        currentTilemapTag = tilemapTag;
        InitializeGraph();
        CacheAllNodeNeighbors();
    }

    public void CacheAllNodeNeighbors()
    {
        foreach (Node node in graph) 
        {
            node.ClearNeighbors();

            Vector2Int[] directions = { Vector2Int.up, Vector2Int.down, Vector2Int.left, Vector2Int.right,
                                        new Vector2Int(1, 1), new Vector2Int(-1, -1), new Vector2Int(1, -1), new Vector2Int(-1, 1) };

            foreach (var direction in directions)
            {
                Node neighbor = GetNode(node.graphPosition + direction);

                if (neighbor != null)
                {
                    node.neighbors.Add(neighbor);
                }
            }
        }
    }
}
