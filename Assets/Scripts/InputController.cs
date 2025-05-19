using UnityEngine;
using UnityEngine.Tilemaps;

public class InputController : MonoBehaviour
{
    public Tilemap tilemap;
    public Sprite startTileSprite;
    public Sprite endTileSprite;
    public Sprite visitedTileSprite;
    public Sprite obstacleTileSprite;
    public Sprite pathTileSprite;
    public Sprite currentTileSprite;
    void Update()
    {
        if (Input.GetMouseButton(0)) // Left click
        {
            Vector3 mouseWorldPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            Vector3Int cellPos = tilemap.WorldToCell(mouseWorldPos);

            if (Input.GetKey(KeyCode.S))
            {
                Tile tile = ScriptableObject.CreateInstance<Tile>();
                tile.sprite = startTileSprite;
                tilemap.SetTile(cellPos, tile);
            }
            else if (Input.GetKey(KeyCode.E))
            {
                Tile tile = ScriptableObject.CreateInstance<Tile>();
                tile.sprite = endTileSprite;
                tilemap.SetTile(cellPos, tile);
            }
            else if (Input.GetKey(KeyCode.O))
            {
                Tile tile = ScriptableObject.CreateInstance<Tile>();
                tile.sprite = obstacleTileSprite;
                tilemap.SetTile(cellPos, tile);
            }
        }
    }
}
