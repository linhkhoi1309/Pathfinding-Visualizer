using UnityEngine;
using UnityEngine.UI;
using TMPro;
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
        AudioSource.PlayClipAtPoint(buttonClickSound, Camera.main.transform.position);
        pathFinder.ResetPathFinding();
    }

    private void OnVisualizeButtonClicked()
    {
        AudioSource.PlayClipAtPoint(buttonClickSound, Camera.main.transform.position);
        pathFinder.InitializePathfinder(pathfindingAlgo);
    }

    private void OnAlgorithmDropdownSelected(int index)
    {
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

    private void Update()
    {
        UpdateUI();
    }

    public void UpdateUI()
    {
        UpdateNumOfNodesExplored(pathFinder.numOfNodesExplored);
        UpdateProcessingTime(pathFinder.processingTime);
        UpdateTotalCost(pathFinder.totalCost);
    }

    private void UpdateProcessingTime(float processingTime)
    {
        processingTimeText.text = "Processing time: " + processingTime.ToString("F2") + " secs";
    }

    public void UpdateNumOfNodesExplored(int num)
    {
        numOfNodesExploredText.text = "Number of nodes explored: " + num;
    }

    public void UpdateTotalCost(int cost)
    {
        totalCostText.text = "Total cost: " + cost;
    }
}
