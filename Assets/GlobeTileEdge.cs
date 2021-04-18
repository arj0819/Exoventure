using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobeTileEdge
{
    public static List<GlobeTileEdge> AllEdges = new List<GlobeTileEdge>();
    public static int TotalGlobeTileEdgeCount = 0;
    public int id = TotalGlobeTileEdgeCount;
    public Vector3 vertex1;
    public Vector3 vertex2;
    public Vector3 motion;
    public float pressure = 0f;
    public float sheer = 0f;

    public GlobeTileEdge(Vector3 vertex1, Vector3 vertex2)
    {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        TotalGlobeTileEdgeCount++;
        AllEdges.Add(this);
    }
}
