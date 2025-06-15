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

    [SerializeField] private TMP_Dropdown mazeDropdown;
    [SerializeField] private AudioClip buttonClickSound;
    PathFinder pathFinder;
    GraphController graphController;
    PathfindingAlgo pathfindingAlgo;

    private void Awake()
    {
        resetButton.onClick.AddListener(OnResetButtonClicked);
        visualizeButton.onClick.AddListener(OnVisualizeButtonClicked);
        algorithmDropdown.onValueChanged.AddListener(OnAlgorithmDropdownSelected);
        mazeDropdown.onValueChanged.AddListener(OnMazeDropdownSelected);
        pathFinder = FindFirstObjectByType<PathFinder>();
        graphController = FindFirstObjectByType<GraphController>();
    }

    private void Start() {
        mazeDropdown.value = 1;
    }

    private void OnMazeDropdownSelected(int index)
    {
        pathFinder.ResetPathFindingConfigs();
        switch (index)
        {
            case 0:
                graphController.LoadTilemap(GlobalConfigs.CUSTOM_TILEMAP);
                break;
            case 1:
                graphController.LoadTilemap(GlobalConfigs.MAZE_1_TILEMAP);
                break;
        }
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
        pathFinder.ResetPathFindingConfigs();
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

    public void UpdateTotalCost(float cost)
    {
        totalCostText.text = "Total cost: " + cost.ToString("F2");
    }
}
