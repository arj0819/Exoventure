using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointsOnSphere : MonoBehaviour
{
    public int points = 4536;
    // Start is called before the first frame update
    void Start()
    {
        float scaling = 10;
        Vector3[] pts = GeneratePointsOnSphere(points);
        List<GameObject> uspheres = new List<GameObject>();
        int i = 0;

        foreach (Vector3 value in pts)
        {
            uspheres.Add(GameObject.CreatePrimitive(PrimitiveType.Sphere));
            uspheres[i].transform.parent = transform;
            uspheres[i].transform.position = value * scaling;
            uspheres[i].transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
            i++;
        }
    }

    // Update is called once per frame
    void Update()
    {

    }

    Vector3[] GeneratePointsOnSphere(int n)
    {
        List<Vector3> upts = new List<Vector3>();
        float inc = Mathf.PI * (3 - Mathf.Sqrt(5));
        float off = 2f / n;
        float x = 0;
        float y = 0;
        float z = 0;
        float r = 0;
        float phi = 0;

        for (var k = 0; k < n; k++)
        {
            y = k * off - 1 + (off / 2);
            r = Mathf.Sqrt(1 - y * y);
            phi = k * inc;
            x = Mathf.Cos(phi) * r;
            z = Mathf.Sin(phi) * r;

            upts.Add(new Vector3(x, y, z));
        }
        Vector3[] pts = upts.ToArray();
        return pts;
    }
}
