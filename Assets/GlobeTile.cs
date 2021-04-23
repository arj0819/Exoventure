using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GlobeTile
{
    public static int GlobeTileCount = 0;
    public int id = GlobeTileCount;
    public static List<GlobeTile> AllTiles = new List<GlobeTile>();

    private static GK.ConvexHullCalculator ConvexHullCalculator = new GK.ConvexHullCalculator();

    public Vector3 delaunayPoint;
    public List<Vector3> vertices = new List<Vector3>();
    public List<Vector3> hullVertices = new List<Vector3>();
    public List<int> triangles = new List<int>();
    public List<Vector3> normals = new List<Vector3>();
    public List<GlobeTile> neighborTiles = new List<GlobeTile>();
    public List<GlobeTileEdge> edges = new List<GlobeTileEdge>();
    public float scale;
    public float phi;
    public float theta;

    public List<Vector3> meshVertices = new List<Vector3>();
    public Mesh mesh = new Mesh();
    public GameObject terrain;
    public TectonicPlate tectonicPlate;
    public AtmospherePoint atmospherePoint;
    public int tilesAwayFromPlateSeed = -1;
    public int tilesAwayFromPlatePerimeter = -1;
    public GlobeTile closestTectonicPerimiterTile;
    public float tectonicPressure = float.MinValue;
    public float tectonicSheer = float.MinValue;
    public Vector3 motion;
    public float elevation;
    public float moisture;
    public float temperature;

    public float shuffleValue = Random.value;

    public GlobeTile(Vector3 delaunayPoint, List<Vector3> vertices, float scale, float phi, float theta)
    {
        GlobeTileCount++;

        this.delaunayPoint = delaunayPoint;
        this.vertices = new List<Vector3>(vertices);
        this.meshVertices = new List<Vector3>(vertices);
        this.meshVertices.Add(delaunayPoint);
        this.scale = scale;
        this.phi = phi;
        this.theta = theta;
        this.moisture = (((-0.5f * Mathf.Cos(6f * this.phi)) + 0.5f)) + Mathf.Cos(24f * this.theta) / 15f;

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
        Vector2[] uvs = UV.UvCalculator.CalculateUVs(this.hullVertices.ToArray(), 1f);
        this.mesh.uv = uvs;

        terrain = GameObject.CreatePrimitive(PrimitiveType.Quad);
        GameObject globe = GameObject.Find("Globe");
        terrain.transform.parent = globe.transform;

        //terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("WaterMaterial").GetComponent<MeshRenderer>().material);
        //terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("DesertMaterial").GetComponent<MeshRenderer>().material);
        //terrain.GetComponent<MeshRenderer>().material.color = new Color(Random.value, Random.value, Random.value);
        terrain.GetComponent<MeshFilter>().sharedMesh = mesh;

        AllTiles.Add(this);
    }
    public static void ShuffleAllTiles()
    {
        for (int i = 0; i < AllTiles.Count; i++)
        {
            AllTiles[i].shuffleValue = Random.value;
        }
    }
    public static void ShuffleTiles(List<GlobeTile> tiles)
    {
        for (int i = 0; i < tiles.Count; i++)
        {
            tiles[i].shuffleValue = Random.value;
        }
        tiles.Sort((a, b) => a.shuffleValue.CompareTo(b.shuffleValue));
    }
}
