using UnityEngine;

public class Node
{
    public Vector2Int graphPosition;
    public Node parentNode;
    public bool isPassable = true;
    public Node(Vector2Int graphPosition)
    {
        this.graphPosition = graphPosition;
        this.parentNode = null;
    }
}
