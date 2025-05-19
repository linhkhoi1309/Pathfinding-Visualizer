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

    private void Awake()
    {
        resetButton.onClick.AddListener(OnResetButtonClicked);
        visualizeButton.onClick.AddListener(OnVisualizeButtonClicked);
    }

    private void OnResetButtonClicked()
    {
        // Logic to reset the grid and clear the path
        Debug.Log("Reset button clicked");
    }
    
    private void OnVisualizeButtonClicked()
    {
        // Logic to start the pathfinding visualization
        Debug.Log("Visualize button clicked");
    }
}
