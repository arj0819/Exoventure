using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CentroidTileTriangularFragment
{
    public List<Vector3> vertices = new List<Vector3>();
    public List<int> triangles = new List<int>();
    public List<Vector3> normals = new List<Vector3>();

    public Mesh mesh = new Mesh();

    public CentroidTileTriangularFragment()
    {

    }
    public CentroidTileTriangularFragment(List<Vector3> vertices, List<int> triangles)
    {
        this.vertices = vertices;
        this.triangles = triangles;

        this.mesh.SetVertices(this.vertices);
        this.mesh.SetTriangles(this.triangles, 0);
        this.mesh.RecalculateNormals();
        this.mesh.GetNormals(this.normals);

        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);

        quad.GetComponent<MeshRenderer>().material.color = new Color(Random.value, Random.value, Random.value);
        quad.GetComponent<MeshFilter>().sharedMesh = mesh;
    }
}
