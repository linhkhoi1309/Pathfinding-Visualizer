using System;
using UnityEngine;

public class Node : IComparable<Node>
{
    public Vector2Int graphPosition;
    public Node parentNode;
    public bool isPassable = true; // Indicates if the node is passable or an obstacle
    public Node(Vector2Int graphPosition)
    {
        this.graphPosition = graphPosition;
        this.parentNode = null;
    }

    public int CompareTo(Node other)
    {
        throw new NotImplementedException();
    }
}
