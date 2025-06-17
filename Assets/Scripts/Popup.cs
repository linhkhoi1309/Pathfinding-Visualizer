using UnityEngine;
using TMPro;

public class Popup : MonoBehaviour
{
    public TextMeshProUGUI titleText;
    public TextMeshProUGUI contentText;
    public void SetPopupContent(string content)
    {
        contentText.text = content;
    }

    public void SetPopupTitle(string title)
    {
        titleText.text = title;
    }
}
