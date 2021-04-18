using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CentroidPoint
{
    public static List<Vector3> All = new List<Vector3>();
    public List<Vector3> delaunayParents;
    public Vector3 centroid;

    public CentroidPoint(List<Vector3> delaunayParents, Vector3 centroid)
    {
        this.delaunayParents = delaunayParents;
        this.centroid = centroid;
        All.Add(centroid);
    }
}
