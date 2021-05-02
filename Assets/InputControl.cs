using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InputControl : MonoBehaviour
{
    public GameObject cameraOrbit;
    public GameObject clickedGameObject;

    public float rotateSpeed = 8f;

    void Update()
    {
        if (Input.GetMouseButton(0))
        {
            float h = rotateSpeed * Input.GetAxis("Mouse X");
            float v = rotateSpeed * Input.GetAxis("Mouse Y");

            if (cameraOrbit.transform.eulerAngles.z + v <= 0.1f || cameraOrbit.transform.eulerAngles.z + v >= 179.9f)
                v = 0;

            cameraOrbit.transform.eulerAngles = new Vector3(cameraOrbit.transform.eulerAngles.x, cameraOrbit.transform.eulerAngles.y + h, cameraOrbit.transform.eulerAngles.z + v);
        }

        if (Input.GetMouseButtonUp(0))
        {
            RaycastHit hitInfo = new RaycastHit();
            bool hit = Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hitInfo);
            if (hit)
            {
                //Debug.Log("Hit " + hitInfo.transform.gameObject.name);
                if (hitInfo.transform.gameObject)
                {
                    Debug.Log(hitInfo.transform.gameObject);
                    this.clickedGameObject = hitInfo.transform.gameObject;
                    GlobeTile clickedTile = GlobeTile.AllTiles.Find(tile => tile.terrain == this.clickedGameObject);
                    if (clickedTile != null)
                    {
                        print("Surface Moisture --> " + clickedTile.surfaceMoisture);
                        print("Air Moisture ------> " + clickedTile.atmospherePoint.airMoisture);
                        print("Elevation ---------> " + clickedTile.elevation);
                        print("Tectonic Pressure -> " + clickedTile.tectonicPressure);
                    }
                }
            }
        }

        float scrollFactor = Input.GetAxis("Mouse ScrollWheel");

        if (scrollFactor != 0)
        {
            cameraOrbit.transform.localScale = cameraOrbit.transform.localScale * (1f - scrollFactor);
        }

    }
}
