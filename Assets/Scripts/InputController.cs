using System.Numerics;
using UnityEngine;
using UnityEngine.Tilemaps;
using Vector3 = UnityEngine.Vector3;

public class InputController : MonoBehaviour
{
    GraphController graphController;

    private void Awake()
    {
        graphController = FindObjectOfType<GraphController>();
    }
    void Update()
    {
        if (Input.GetMouseButton(0)) // Left click
        {
            Vector3 mouseWorldPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            Vector3Int cellPosition = graphController.currentTilemap.WorldToCell(mouseWorldPosition);
            Vector2Int graphPosition = new Vector2Int(cellPosition.x - graphController.gridLowerBound.x, cellPosition.y - graphController.gridLowerBound.y);
            if (Input.GetKey(KeyCode.S))
            {
                graphController.SetStartNode(graphPosition);
            }
            else if (Input.GetKey(KeyCode.E))
            {
                graphController.SetEndNode(graphPosition);
            }
            else if (Input.GetKey(KeyCode.O))
            {
                graphController.SetObstacleNode(graphPosition);
            }
            else if (Input.GetKey(KeyCode.D))
            {
                graphController.ClearNode(graphPosition);
            }
        }
    }
}
