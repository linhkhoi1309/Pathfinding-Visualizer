using System;
using UnityEngine;

public class Node : IComparable<Node>
{
    public Vector2Int graphPosition;
    public Node parentNode;
    public bool isPassable = true; // Indicates if the node is passable or an obstacle
    public float priority;
    public float distanceTraveled = 0f; // Distance traveled from the start node to this node
    public Node(Vector2Int graphPosition)
    {
        this.graphPosition = graphPosition;
        this.parentNode = null;
    }

    public int CompareTo(Node other)
    {
        if (priority < other.priority) return -1;
        else if (priority > other.priority) return 1;
        else return 0;
    }
}
