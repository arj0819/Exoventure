using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Globe : MonoBehaviour
{
    private struct UnitSpherePoint
    {
        public static List<Vector3> AllVectors = new List<Vector3>();
        public static List<UnitSpherePoint> AllUnitSpherePoints = new List<UnitSpherePoint>();
        public Vector3 point { get; }
        public float theta { get; }
        public UnitSpherePoint(Vector3 point, float theta)
        {
            this.point = point;
            this.theta = theta;
            AllVectors.Add(this.point);
            AllUnitSpherePoints.Add(this);
        }
    }

    public int numberOfTiles = 500;
    public float scale = 20;
    public float jitter = 0;
    public int numberOfTectonicPlates = 15;
    public int tectonicPlateSmoothness = 0;
    public int oceanicRate = 70;
    private List<UnitSpherePoint> tileGraphNodes;
    public static List<GlobeTile> globeTiles = new List<GlobeTile>();
    private List<TectonicPlate> globePlates = new List<TectonicPlate>();
    private bool globeGenerationFinished = false;

    public static Material DesertMaterial;
    public static Material GrasslandMaterial;
    public static Material PlainsMaterial;
    public static Material SnowMaterial;
    public static Material TundraMaterial;
    public static Material WaterMaterial;


    // Start is called before the first frame update
    void Start()
    {
        // Load materials
        DesertMaterial = new Material(GameObject.Find("DesertMaterial").GetComponent<MeshRenderer>().material);
        GrasslandMaterial = new Material(GameObject.Find("GrasslandMaterial").GetComponent<MeshRenderer>().material);
        PlainsMaterial = new Material(GameObject.Find("PlainsMaterial").GetComponent<MeshRenderer>().material);
        SnowMaterial = new Material(GameObject.Find("SnowMaterial").GetComponent<MeshRenderer>().material);
        TundraMaterial = new Material(GameObject.Find("TundraMaterial").GetComponent<MeshRenderer>().material);
        WaterMaterial = new Material(GameObject.Find("WaterMaterial").GetComponent<MeshRenderer>().material);

        // Initialize 
        tileGraphNodes = GenerateTileGraphNodes(numberOfTiles);
        List<GameObject> tileGraphNodeSpheres = new List<GameObject>();
        List<GameObject> centroidTileVertexSpheres = new List<GameObject>();

        GK.ConvexHullCalculator convexHullCalculator = new GK.ConvexHullCalculator();
        var delaunayVerticies = new List<Vector3>();
        var delaunayTriangles = new List<int>();
        var delaunayNormals = new List<Vector3>();

        var centroids = new List<CentroidPoint>();
        var centroidVerticies = new List<Vector3>();
        var centroidTriangles = new List<int>();
        var centroidNormals = new List<Vector3>();

        // Generate delaunay convex hull
        convexHullCalculator.GenerateHull(UnitSpherePoint.AllVectors, true, ref delaunayVerticies, ref delaunayTriangles, ref delaunayNormals);

        // Determine centroids of each delaunay triangle on the convex hull
        Vector3 firstDelaunayTriangleVertex = new Vector3();
        Vector3 secondDelaunayTriangleVertex = new Vector3();
        Vector3 thirdDelaunayTriangleVertex = new Vector3();
        List<Vector3> delaunayVertexSeeds = new List<Vector3>();

        for (int i = 0; i < delaunayTriangles.Count; i++)
        {
            if (i % 3 == 0)
            {
                firstDelaunayTriangleVertex = delaunayVerticies[i];
                delaunayVertexSeeds.Add(delaunayVerticies[i]);
            } else if (i % 3 == 1) {
                secondDelaunayTriangleVertex = delaunayVerticies[i];
                delaunayVertexSeeds.Add(delaunayVerticies[i]);
            } else if (i % 3 == 2) {
                thirdDelaunayTriangleVertex = delaunayVerticies[i];
                delaunayVertexSeeds.Add(delaunayVerticies[i]);

                float firstDelaunayTriangleVertexX = firstDelaunayTriangleVertex.x;
                float firstDelaunayTriangleVertexY = firstDelaunayTriangleVertex.y;
                float firstDelaunayTriangleVertexZ = firstDelaunayTriangleVertex.z;
                float secondDelaunayTriangleVertexX = secondDelaunayTriangleVertex.x;
                float secondDelaunayTriangleVertexY = secondDelaunayTriangleVertex.y;
                float secondDelaunayTriangleVertexZ = secondDelaunayTriangleVertex.z;
                float thirdDelaunayTriangleVertexX = thirdDelaunayTriangleVertex.x;
                float thirdDelaunayTriangleVertexY = thirdDelaunayTriangleVertex.y;
                float thirdDelaunayTriangleVertexZ = thirdDelaunayTriangleVertex.z;

                float averageX = (firstDelaunayTriangleVertexX + secondDelaunayTriangleVertexX + thirdDelaunayTriangleVertexX) / 3;
                float averageY = (firstDelaunayTriangleVertexY + secondDelaunayTriangleVertexY + thirdDelaunayTriangleVertexY) / 3;
                float averageZ = (firstDelaunayTriangleVertexZ + secondDelaunayTriangleVertexZ + thirdDelaunayTriangleVertexZ) / 3;

                centroids.Add(new CentroidPoint(new List<Vector3>(delaunayVertexSeeds), new Vector3(averageX, averageY, averageZ)));
                delaunayVertexSeeds.Clear();
            }
        }

        // Generate the centroidal voronoi convex hull
        convexHullCalculator.GenerateHull(CentroidPoint.All, true, ref centroidVerticies, ref centroidTriangles, ref centroidNormals);
        print(CentroidPoint.All.Count);

        //Determine each tile
        List<Vector3> tileVertices = new List<Vector3>();
        for (int i = 0; i < centroids.Count; i++)
        {
            CentroidPoint currentCentroidPoint = centroids[i];
            for (int p = 0; p < currentCentroidPoint.delaunayParents.Count; p++)
            {
                Vector3 currentCentroidPointParent = currentCentroidPoint.delaunayParents[p];
                for (int j = 0; j < centroids.Count; j++)
                {
                    CentroidPoint nextCentroidPoint = centroids[j];
                    for (int k = 0; k < nextCentroidPoint.delaunayParents.Count; k++)
                    {
                        Vector3 nextCentroidPointCurrentDelaunayParent = nextCentroidPoint.delaunayParents[k];
                        if (currentCentroidPointParent == nextCentroidPointCurrentDelaunayParent)
                        {
                            tileVertices.Add(nextCentroidPoint.centroid);
                        }
                    }
                }
                if (globeTiles.Find(tile => tile.delaunayPoint == currentCentroidPointParent) == null)
                {
                    Vector3 globeTileVector = new Vector3(currentCentroidPointParent.x, currentCentroidPointParent.y, currentCentroidPointParent.z);
                    Vector3 globeTileMatchingUnitSpherePoint = UnitSpherePoint.AllVectors.Find(point => point.Equals(globeTileVector));
                    globeTiles.Add(new GlobeTile(globeTileVector, new List<Vector3>(tileVertices), scale, UnitSpherePoint.AllUnitSpherePoints[UnitSpherePoint.AllVectors.IndexOf(globeTileVector)].theta));
                }
                tileVertices.Clear();
            }
        }
        print(globeTiles.Count);


        // Determine each tile's neighbors and edges
        List<GlobeTile> neighboringTiles = new List<GlobeTile>();
        globeTiles = new List<GlobeTile>(globeTiles.OrderBy(tile => Vector3.Distance(globeTiles[0].delaunayPoint, tile.delaunayPoint)));
        for (int i = 0; i < globeTiles.Count; i++)
        {
            GlobeTile currentGlobeTile = globeTiles[i];
            for (int j = 0; j < globeTiles.Count; j++)
            {
                GlobeTile neighboringGlobeTile = globeTiles[j];
                neighboringTiles.Add(neighboringGlobeTile);
            }
            neighboringTiles = new List<GlobeTile>(neighboringTiles.OrderBy(tile => (Vector3.Distance(currentGlobeTile.delaunayPoint, tile.delaunayPoint))).ToList());
            neighboringTiles.RemoveAt(0);
            int currentGlobeTileNeighborCount = currentGlobeTile.vertices.Count;
            for (int j = 0; j < currentGlobeTileNeighborCount; j++)
            {
                List<Vector3> commonVertices = currentGlobeTile.vertices.Intersect(neighboringTiles[j].vertices).ToList();
                // Make sure that the neighboring tile has at least 2 common vertices, otherwise remove it from consideration
                while (commonVertices.Count != 2)
                {
                    neighboringTiles.Remove(neighboringTiles[j]);
                    commonVertices = currentGlobeTile.vertices.Intersect(neighboringTiles[j].vertices).ToList();
                }
                currentGlobeTile.neighborTiles.Add(neighboringTiles[j]);
                GlobeTileEdge potentialMatchingEdge = GlobeTileEdge.AllEdges.Find(edge => commonVertices.Contains(edge.vertex1) && commonVertices.Contains(edge.vertex2));
                if (potentialMatchingEdge != null)
                {
                    currentGlobeTile.edges.Add(potentialMatchingEdge);
                } else {
                    currentGlobeTile.edges.Add(new GlobeTileEdge(commonVertices[0], commonVertices[1]));
                }
                commonVertices.Clear();
            }
            neighboringTiles.Clear();
        }

        // Generate Tectonic Plate Seeds
        List<GlobeTile> tectonicPlateSeeds = new List<GlobeTile>();
        List<GlobeTile> chosenGlobeTiles = new List<GlobeTile>();
        for (int i = 0; i < numberOfTectonicPlates; i++)
        {
            int seedIndex = Random.Range(0, globeTiles.Count);
            while (tectonicPlateSeeds.Contains(globeTiles[seedIndex]) || tectonicPlateSeeds.Find(seed => seed.vertices.Intersect(globeTiles[seedIndex].vertices).ToList().Count == 2) != null)
            {
                seedIndex = Random.Range(0, globeTiles.Count);
            }
            tectonicPlateSeeds.Add(globeTiles[seedIndex]);
            chosenGlobeTiles.Add(globeTiles[seedIndex]);
            globeTiles[seedIndex].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("DesertMaterial").GetComponent<MeshRenderer>().material);
        }

        // Build Tectonic Plates
        List<TectonicPlate> tectonicPlates = new List<TectonicPlate>();
        for (int i = 0; i < tectonicPlateSeeds.Count; i++)
        {
            tectonicPlates.Add(new TectonicPlate(tectonicPlateSeeds[i], tectonicPlateSmoothness, oceanicRate));
        }
        List<int> finishedPlateIndices = new List<int>();
        while (finishedPlateIndices.Count < tectonicPlates.Count)
        {
            for (int i = 0; i < tectonicPlates.Count; i++)
            {
                if (!tectonicPlates[i].FindNextTiles())
                {
                    if (!finishedPlateIndices.Contains(i))
                    {
                        finishedPlateIndices.Add(i);
                    }
                }
            }
        }


        // print each plate perimeter tile count
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            //print(tectonicPlates[i].perimeterTiles.Count);
            //print(tectonicPlates[i].perimeterTileNeighbors.Count);
            //print(tectonicPlates[i].perimeterEdges.Count);
        }

        // print each plate perimeter tile count
        //for (int i = 0; i < tectonicPlates.Count; i++)
        //{
        //    // Determine perimeter tiles of the tectonic plate
        //    TectonicPlate currentPlate = tectonicPlates[i];
        //    for (int j = 0; j < currentPlate.plateTiles.Count; j++)
        //    {
        //        GlobeTile currentPlateTile = currentPlate.plateTiles[j];
        //        for (int k = 0; k < currentPlateTile.neighborTiles.Count; k++)
        //        {
        //            GlobeTile currentPlateTileNeighbor = currentPlateTile.neighborTiles[k];
        //            if (currentPlateTileNeighbor.tectonicPlate == null || currentPlateTileNeighbor.tectonicPlate.id != currentPlate.id)
        //            {
        //                if (!currentPlate.perimeterTiles.Any(tile => tile.delaunayPoint == currentPlateTile.delaunayPoint))
        //                {
        //                    currentPlate.perimeterTiles.Add(currentPlateTile);
        //                }
        //            }
        //        }
        //    }
        //    print(tectonicPlates[i].perimeterTiles.Count);
        //}

        // Visualise tile motion vectors
        for (int i = 0; i < globeTiles.Count; i++)
        {
            LineRenderer line = new GameObject().AddComponent<LineRenderer>();

            line.startWidth = 0.2f;
            line.endWidth = 0f;
            line.startColor = Color.magenta;
            line.endColor = Color.magenta;
            line.positionCount = 2;

            Vector3 tileDelaunayPointScaled = globeTiles[i].delaunayPoint * scale * 1.005f;
            Vector3 tileMotionEndPoint = tileDelaunayPointScaled + globeTiles[i].motion;

            line.SetPosition(0, tileDelaunayPointScaled);
            line.SetPosition(1, tileMotionEndPoint);
        }

        // Visualise tile eges
        List<GlobeTileEdge> globeEdges = GlobeTileEdge.AllEdges;
        for (int j = 0; j < globeEdges.Count; j++)
        {
            GlobeTileEdge currentGlobeTileEdge = globeEdges[j];
            LineRenderer line = new GameObject().AddComponent<LineRenderer>();
            line.material = Globe.DesertMaterial;
            line.material.color = Color.black;
            line.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

            line.startWidth = 0.05f;
            line.endWidth = 0.05f;
            line.startColor = Color.black;
            line.endColor = Color.black;
            line.positionCount = 2;

            Vector3 linePosition1 = currentGlobeTileEdge.vertex1 * scale * 1.0005f;
            Vector3 linePosition2 = currentGlobeTileEdge.vertex2 * scale * 1.0005f;
            line.SetPosition(0, linePosition1);
            line.SetPosition(1, linePosition2);
        }
        print(globeEdges.Count);

        // print each plate perimeter tile count
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            TectonicPlate currentPlate = tectonicPlates[i];
            for (int j = 0; j < currentPlate.perimeterEdges.Count; j++)
            {
                GlobeTileEdge currentPlateTileEdge = currentPlate.perimeterEdges[j];
                LineRenderer line = new GameObject().AddComponent<LineRenderer>();
                line.material = Globe.DesertMaterial;
                line.material.color = new Color(0f, 255f,0f);
                line.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

                line.startWidth = 0.15f;
                line.endWidth = 0.15f;
                line.startColor = Color.black;
                line.endColor = Color.black;
                line.positionCount = 2;

                Vector3 linePosition1 = currentPlateTileEdge.vertex1 * scale * 1.0006f;
                Vector3 linePosition2 = currentPlateTileEdge.vertex2 * scale * 1.0006f;
                line.SetPosition(0, linePosition1);
                line.SetPosition(1, linePosition2);
            }
        }









        // Failed tectonic plate generation algorithm
        //List<List<GlobeTile>> tectonicPlateTiles = new List<List<GlobeTile>>();
        //for (int i = 0; i < tectonicPlateSeeds.Count; i++)
        //{
        //    tectonicPlateTiles.Add(new List<GlobeTile>());
        //    tectonicPlateTiles[i].Add(tectonicPlateSeeds[i]);
        //}

        //while (chosenGlobeTiles.Count < globeTiles.Count)
        //{
        //    for (int i = 0; i < tectonicPlateTiles.Count; i++)
        //    {
        //        List<GlobeTile> currentPlate = tectonicPlateTiles[i];
        //        List<GlobeTile> possibleNeighborSeeds = new List<GlobeTile>();
        //        possibleNeighborSeeds = possibleNeighborSeeds.Concat(currentPlate).ToList();
        //        for (int j = 0; j < currentPlate.Count; j++)
        //        {
        //            print("here");
        //            possibleNeighborSeeds = possibleNeighborSeeds.Concat(currentPlate[j].neighborTiles).ToList();
        //        }
        //        possibleNeighborSeeds = possibleNeighborSeeds.Except(chosenGlobeTiles).ToList();
        //        GlobeTile neighborSeedTile = possibleNeighborSeeds[Random.Range(0, possibleNeighborSeeds.Count)];
        //        List<GlobeTile> neighborSeedTileNeighbors = new List<GlobeTile>(neighborSeedTile.neighborTiles);
        //        int neighborCount = neighborSeedTileNeighbors.Count;
        //        int randomNeighborIndex = Random.Range(0, neighborCount);
        //        GlobeTile randomNeighbor = neighborSeedTileNeighbors[randomNeighborIndex];

        //        while (neighborCount > 0 || chosenGlobeTiles.Find(tile => tile.delaunayPoint.Equals(randomNeighbor.delaunayPoint)) != null)
        //        {
        //            neighborSeedTileNeighbors.RemoveAt(randomNeighborIndex);
        //            print("here");
        //            neighborCount = neighborSeedTileNeighbors.Count;
        //            print("here");
        //            if (neighborCount == 0)
        //            {
        //                break;
        //            }
        //            randomNeighborIndex = Random.Range(0, neighborCount);
        //            print(randomNeighborIndex);
        //            randomNeighbor = neighborSeedTileNeighbors[randomNeighborIndex];

        //        }
        //        currentPlate.Add(randomNeighbor);
        //        chosenGlobeTiles.Add(randomNeighbor);
        //    }
        //}

        //for (int i = 0; i < tectonicPlateTiles.Count; i++)
        //{
        //    globePlates.Add(new TectonicPlate(tectonicPlateTiles[i]));
        //}






        //globeTiles[199].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("Globe").GetComponent<MeshRenderer>().material);
        //for (int i = 0; i < globeTiles[199].neighborTiles.Count; i++)
        //{
        //    globeTiles[199].neighborTiles[i].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("WaterMaterial").GetComponent<MeshRenderer>().material);
        //}

        // Add spheres to the vertices of the Centroid Tiles
        //for (int i = 0; i < centroidVerticies.Count; i++)
        //{
        //    centroidTileVertexSpheres.Add(GameObject.CreatePrimitive(PrimitiveType.Sphere));
        //    centroidTileVertexSpheres[i].transform.parent = transform;
        //    centroidTileVertexSpheres[i].transform.position = centroidVerticies[i] * scale;
        //    centroidTileVertexSpheres[i].transform.localScale = new Vector3(0.001f, 0.001f, 0.001f);
        //    centroidTileVertexSpheres[i].transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
        //}

        //// Add spheres to the vertices of the Delaunay Triangles
        //for (int i = 0; i < delaunayVerticies.Count; i++)
        //{
        //    tileGraphNodeSpheres.Add(GameObject.CreatePrimitive(PrimitiveType.Sphere));
        //    tileGraphNodeSpheres[i].transform.parent = transform;
        //    tileGraphNodeSpheres[i].transform.position = delaunayVerticies[i] * scale;
        //    tileGraphNodeSpheres[i].transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);
        //}









        // Draw lines from each point to the closest neighboring point
        //List<GameObject> tileGraphNodeSphereCopy = new List<GameObject>(tileGraphNodeSpheres);
        //for (int i = 0; i < tileGraphNodeSpheres.Count; i++)
        //{
        //    if (i > 0)
        //    {
        //        LineRenderer line = new GameObject().AddComponent<LineRenderer>();

        //        line.startWidth = 0.02f;
        //        line.endWidth = 0.02f;
        //        line.startColor = Color.red;
        //        line.endColor = Color.red;
        //        line.positionCount = 2;

        //        GameObject currentSphere = tileGraphNodeSphereCopy[i];
        //        GameObject closestSphere = tileGraphNodeSphereCopy[i - 1];
        //        //find closest sphere
        //        for (int k = 0; k < tileGraphNodeSphereCopy.Count; k++)
        //        {
        //            float distanceToClosestSphere = Vector3.Distance(currentSphere.transform.position, closestSphere.transform.position);
        //            float distanceToPotentialClosestSphere = Vector3.Distance(currentSphere.transform.position, tileGraphNodeSphereCopy[k].transform.position);

        //            if (distanceToPotentialClosestSphere > 0f)
        //            {
        //                if (distanceToPotentialClosestSphere < distanceToClosestSphere)
        //                {
        //                    closestSphere = tileGraphNodeSphereCopy[k];
        //                }
        //            }
        //        }

        //        line.SetPosition(0, closestSphere.transform.position);
        //        line.SetPosition(1, currentSphere.transform.position);
        //    }
        //}
        globeGenerationFinished = true;
    }

    private float nextActionTime = 0f;
    public float period = 1f;
    private int tileCounter = 2999;
    // Update is called once per frame
    void Update()
    {
        //if (globeGenerationFinished)
        //{
        //    if (Time.time > nextActionTime)
        //    {
        //        if (tileCounter == 2999)
        //        {
        //            globeTiles[tileCounter].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("Globe").GetComponent<MeshRenderer>().material);
        //            for (int i = 0; i < globeTiles[tileCounter].neighborTiles.Count; i++)
        //            {
        //                globeTiles[tileCounter].neighborTiles[i].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("WaterMaterial").GetComponent<MeshRenderer>().material);
        //            }
        //        }
        //        else
        //        {
        //            globeTiles[tileCounter + 1].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("LandMaterial").GetComponent<MeshRenderer>().material);
        //            for (int i = 0; i < globeTiles[tileCounter + 1].neighborTiles.Count; i++)
        //            {
        //                globeTiles[tileCounter + 1].neighborTiles[i].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("LandMaterial").GetComponent<MeshRenderer>().material);
        //            }
        //            globeTiles[tileCounter].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("Globe").GetComponent<MeshRenderer>().material);
        //            for (int i = 0; i < globeTiles[tileCounter].neighborTiles.Count; i++)
        //            {
        //                globeTiles[tileCounter].neighborTiles[i].terrain.GetComponent<MeshRenderer>().material = new Material(GameObject.Find("WaterMaterial").GetComponent<MeshRenderer>().material);
        //            }
        //        }
        //        nextActionTime += period;
        //        tileCounter--;
        //    }
        //}
    }
    List<UnitSpherePoint> GenerateTileGraphNodes(int numberOfTiles)
    {
        List<UnitSpherePoint> tileGraphNodes = new List<UnitSpherePoint>();
        float nodeSeparationAngle = Mathf.PI * (3 - Mathf.Sqrt(5)); //roughly 2.4f always
        float deltaTheta = 2f / numberOfTiles;
        float nodeTheta = 0;
        float nodePositionX = 0;
        float nodePositionY = 0;
        float nodePositionZ = 0;
        float nodeDistanceFromGlobeOrigin = 0;
        float nodePhi = 0;

        for (var currentNodeNumber = 0; currentNodeNumber <= numberOfTiles; currentNodeNumber++)
        {

            if (jitter > 100)
            {
                jitter = 100;
            }
            var jitterPhiDirection = Random.value > 0.5f ? 1 : -1;
            var jitterPhi = ((Random.value / (numberOfTiles)) * jitter) * jitterPhiDirection;

            nodePositionY = ((currentNodeNumber * deltaTheta) - 1) + (deltaTheta / 2);
            nodeDistanceFromGlobeOrigin = Mathf.Sqrt(1 - Mathf.Pow(nodePositionY, 2));
            nodePhi = currentNodeNumber * nodeSeparationAngle + jitterPhi;
            nodePositionX = Mathf.Cos(nodePhi) * nodeDistanceFromGlobeOrigin;
            nodePositionZ = Mathf.Sin(nodePhi) * nodeDistanceFromGlobeOrigin;
            nodePhi -= jitterPhi;
            nodeTheta += deltaTheta;

            tileGraphNodes.Add(new UnitSpherePoint(new Vector3(nodePositionX, nodePositionY , nodePositionZ), nodeTheta));
        }

        return tileGraphNodes;
    }
}
