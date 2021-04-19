using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobeTileEdge
{
    public static List<GlobeTileEdge> AllEdges = new List<GlobeTileEdge>();
    public List<GlobeTile> adjacentTiles = new List<GlobeTile>();
    public static int TotalGlobeTileEdgeCount = 0;
    public int id = TotalGlobeTileEdgeCount;
    public Vector3 vertex1;
    public Vector3 vertex2;
    public Vector3 edgeVector;
    public float tectonicPressure = 0f;
    public float tectonicSheer = 0f;

    public GlobeTileEdge(Vector3 vertex1, Vector3 vertex2, List<GlobeTile> adjacentTiles)
    {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.edgeVector = (this.vertex2 - this.vertex1).normalized;
        this.adjacentTiles = adjacentTiles;
        TotalGlobeTileEdgeCount++;
        AllEdges.Add(this);
    }
}
