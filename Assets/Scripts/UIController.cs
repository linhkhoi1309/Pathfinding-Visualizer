using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.IO;
public class UIController : MonoBehaviour
{
    [SerializeField] private Button resetButton;
    [SerializeField] private Button visualizeButton;
    [SerializeField] private TextMeshProUGUI processingTimeText;
    [SerializeField] private TextMeshProUGUI numOfNodesExploredText;
    [SerializeField] private TextMeshProUGUI totalCostText;
    [SerializeField] private TMP_Dropdown algorithmDropdown;
    [SerializeField] private AudioClip buttonClickSound;
    PathFinder pathFinder;

    PathfindingAlgo pathfindingAlgo;

    private void Awake()
    {
        resetButton.onClick.AddListener(OnResetButtonClicked);
        visualizeButton.onClick.AddListener(OnVisualizeButtonClicked);
        algorithmDropdown.onValueChanged.AddListener(OnAlgorithmDropdownSelected);
        pathFinder = FindObjectOfType<PathFinder>();
    }

    private void OnResetButtonClicked()
    {
        Debug.Log("Reset button clicked");
        AudioSource.PlayClipAtPoint(buttonClickSound, Camera.main.transform.position);
        pathFinder.ResetPathfinding();
    }

    private void OnVisualizeButtonClicked()
    {
        Debug.Log("Visualize button clicked");
        AudioSource.PlayClipAtPoint(buttonClickSound, Camera.main.transform.position);
        pathFinder.InitializePathfinder(pathfindingAlgo);
    }

    private void OnAlgorithmDropdownSelected(int index)
    {
        Debug.Log("Algorithm selected: " + algorithmDropdown.options[index].text);
        switch (index)
        {
            case 0:
                pathfindingAlgo = PathfindingAlgo.DFS;
                break;
            case 1:
                pathfindingAlgo = PathfindingAlgo.BFS;
                break;
            case 2:
                pathfindingAlgo = PathfindingAlgo.UCS;
                break;
            case 3:
                pathfindingAlgo = PathfindingAlgo.AStar;
                break;
            case 4:
                pathfindingAlgo = PathfindingAlgo.GreedyBestFirstSearch;
                break;
            case 5:
                pathfindingAlgo = PathfindingAlgo.IDAStar;
                break;
            case 6:
                pathfindingAlgo = PathfindingAlgo.IDDFS;
                break;
            case 7:
                pathfindingAlgo = PathfindingAlgo.BidirectionalSearch;
                break;
        }
    }

    private void Update() {
        
    }
}
