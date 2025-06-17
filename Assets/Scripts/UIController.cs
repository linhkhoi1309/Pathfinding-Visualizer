using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;
using DG.Tweening;
public class UIController : MonoBehaviour
{
    [SerializeField] private Button resetButton;
    [SerializeField] private Button visualizeButton;
    [SerializeField] private TextMeshProUGUI processingTimeText;
    [SerializeField] private TextMeshProUGUI numOfNodesExploredText;
    [SerializeField] private TextMeshProUGUI memoryUsageText;
    [SerializeField] private TextMeshProUGUI totalCostText;
    [SerializeField] private TMP_Dropdown algorithmDropdown;

    [SerializeField] private TMP_Dropdown mazeDropdown;
    [SerializeField] private AudioClip buttonClickSound;
    [SerializeField] private GameObject popupPrefab;
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

    private void Start()
    {
        mazeDropdown.value = 1;
        ShowPopup("Welcome to Pathfinding Visualizer", "Select a maze and an algorithm to visualize the pathfinding process. Click 'Visualize' to start.");
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
        UpdateMemoryUsage(pathFinder.memoryUsage);
    }

    private void UpdateProcessingTime(float processingTime)
    {
        processingTimeText.text = "Processing time: " + processingTime.ToString("F5") + " s";
    }

    public void UpdateNumOfNodesExplored(int num)
    {
        numOfNodesExploredText.text = "Number of nodes explored: " + num;
    }
    public void UpdateMemoryUsage(long memoryUsage)
    {
        memoryUsageText.text = "Memory usage: " + memoryUsage.ToString() + " B";
    }

    public void UpdateTotalCost(float cost)
    {
        totalCostText.text = "Total cost: " + cost.ToString("F2");
    }

    public void ShowPopup(string title, string content)
    {
        GameObject popup = Instantiate(popupPrefab, transform);
        popup.GetComponent<Popup>().SetPopupTitle(title);
        popup.GetComponent<Popup>().SetPopupContent(content);
        AnimatePopup(popup);
    }

    private void AnimatePopup(GameObject popup)
    {
        RectTransform rectTransform = popup.GetComponent<RectTransform>();
        rectTransform.DOAnchorPosY(-100, 3f).OnComplete(() =>
        {
            Destroy(popup);
        });
    }
}
