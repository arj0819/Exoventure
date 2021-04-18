using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobeTile
{
    public static int GlobeTileCount = 0;
    public int id = GlobeTileCount;
    private static GK.ConvexHullCalculator ConvexHullCalculator = new GK.ConvexHullCalculator();

    public Vector3 delaunayPoint;
    public List<Vector3> vertices = new List<Vector3>();
    public List<Vector3> hullVertices = new List<Vector3>();
    public List<int> triangles = new List<int>();
    public List<Vector3> normals = new List<Vector3>();
    public List<GlobeTile> neighborTiles = new List<GlobeTile>();
    public List<GlobeTileEdge> edges = new List<GlobeTileEdge>();
    public float theta;

    public List<Vector3> meshVertices = new List<Vector3>();
    public Mesh mesh = new Mesh();
    public GameObject terrain = GameObject.CreatePrimitive(PrimitiveType.Quad);
    public TectonicPlate tectonicPlate;
    public Vector3 motion;

    public GlobeTile(Vector3 delaunayPoint, List<Vector3> vertices, float scale, float theta)
    {
        GlobeTileCount++;

        this.delaunayPoint = delaunayPoint;
        this.vertices = new List<Vector3>(vertices);
        this.meshVertices = new List<Vector3>(vertices);
        this.meshVertices.Add(delaunayPoint);
        this.theta = theta;

        for (int i = 0; i < this.meshVertices.Count; i++)
        {
            this.meshVertices[i] *= scale;
        }

        ConvexHullCalculator.GenerateHull(this.meshVertices, true, ref hullVertices, ref this.triangles, ref this.normals);

        this.mesh.SetVertices(this.hullVertices);
        this.mesh.SetTriangles(this.triangles, 0);
        this.mesh.SetNormals(this.normals);
        this.mesh.RecalculateBounds();
        this.mesh.RecalculateNormals();
        this.mesh.RecalculateTangents();

        terrain = GameObject.CreatePrimitive(PrimitiveType.Quad);

        //terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("WaterMaterial").GetComponent<MeshRenderer>().material);
        terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("DesertMaterial").GetComponent<MeshRenderer>().material);
        //terrain.GetComponent<MeshRenderer>().material.color = new Color(Random.value, Random.value, Random.value);
        terrain.GetComponent<MeshFilter>().sharedMesh = mesh;


    }
}
