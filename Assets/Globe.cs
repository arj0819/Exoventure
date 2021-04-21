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
    public static Material EdgeMaterial;

    public static Material WaterElevation10Material;
    public static Material WaterElevation9Material;
    public static Material WaterElevation8Material;
    public static Material WaterElevation7Material;
    public static Material WaterElevation6Material;
    public static Material WaterElevation5Material;
    public static Material WaterElevation4Material;
    public static Material WaterElevation3Material;
    public static Material WaterElevation2Material;
    public static Material WaterElevation1Material;

    public static Material LandElevation10Material;
    public static Material LandElevation9Material;
    public static Material LandElevation8Material;
    public static Material LandElevation7Material;
    public static Material LandElevation6Material;
    public static Material LandElevation5Material;
    public static Material LandElevation4Material;
    public static Material LandElevation3Material;
    public static Material LandElevation2Material;
    public static Material LandElevation1Material;


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
        EdgeMaterial = new Material(GameObject.Find("EdgeMaterial").GetComponent<MeshRenderer>().material);

        WaterElevation10Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[0]);
        WaterElevation9Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[1]);
        WaterElevation8Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[2]);
        WaterElevation7Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[3]);
        WaterElevation6Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[4]);
        WaterElevation5Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[5]);
        WaterElevation4Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[6]);
        WaterElevation3Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[7]);
        WaterElevation2Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[8]);
        WaterElevation1Material = new Material(GameObject.Find("WaterElevationMaterials").GetComponent<MeshRenderer>().materials[9]);

        LandElevation10Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[0]);
        LandElevation9Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[1]);
        LandElevation8Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[2]);
        LandElevation7Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[3]);
        LandElevation6Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[4]);
        LandElevation5Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[5]);
        LandElevation4Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[6]);
        LandElevation3Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[7]);
        LandElevation2Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[8]);
        LandElevation1Material = new Material(GameObject.Find("LandElevationMaterials").GetComponent<MeshRenderer>().materials[9]);

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
                GlobeTile currentNeighboringTile = neighboringTiles[j];
                List<GlobeTile> adjacentTiles = new List<GlobeTile>();
                adjacentTiles.Add(currentGlobeTile);
                adjacentTiles.Add(currentNeighboringTile);
                GlobeTileEdge potentialMatchingEdge = GlobeTileEdge.AllEdges.Find(edge => commonVertices.Contains(edge.vertex1) && commonVertices.Contains(edge.vertex2));
                if (potentialMatchingEdge != null)
                {
                    currentGlobeTile.edges.Add(potentialMatchingEdge);
                } else {
                    currentGlobeTile.edges.Add(new GlobeTileEdge(commonVertices[0], commonVertices[1], adjacentTiles));
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
                if (!finishedPlateIndices.Contains(i))
                {
                    if (!tectonicPlates[i].FindNextTiles())
                    {
                        finishedPlateIndices.Add(i);
                    }
                }
            }
        }

        // Determine the stresses on each edge of each tectonic plate
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            TectonicPlate currentTectonicPlate = tectonicPlates[i];
            for (int j = 0; j < currentTectonicPlate.perimeterEdges.Count; j++)
            {
                GlobeTileEdge currentPerimeterEdge = currentTectonicPlate.perimeterEdges[j];
                Vector3 relativeTileMotion = Vector3.zero;
                Vector3 tectonicPressureVector = Vector3.zero;
                Vector3 tectonicSheerVector = Vector3.zero;

                if (currentPerimeterEdge.adjacentTiles.Count == 2)
                {
                    relativeTileMotion = currentPerimeterEdge.adjacentTiles[0].motion - currentPerimeterEdge.adjacentTiles[1].motion;
                    tectonicSheerVector = Vector3.Dot(relativeTileMotion, currentPerimeterEdge.edgeVector) * currentPerimeterEdge.edgeVector;
                    tectonicPressureVector = tectonicSheerVector - relativeTileMotion;

                    currentPerimeterEdge.tectonicSheer = tectonicSheerVector.magnitude;

                    Vector3 tileDirectionVector1 = currentPerimeterEdge.adjacentTiles[1].delaunayPoint - currentPerimeterEdge.adjacentTiles[0].delaunayPoint;
                    float collisionSeparationDeterminate1 = Vector3.Dot(tileDirectionVector1, currentPerimeterEdge.adjacentTiles[0].motion);
                    Vector3 tileDirectionVector2 = currentPerimeterEdge.adjacentTiles[0].delaunayPoint - currentPerimeterEdge.adjacentTiles[1].delaunayPoint;
                    float collisionSeparationDeterminate2 = Vector3.Dot(tileDirectionVector2, currentPerimeterEdge.adjacentTiles[1].motion);

                    float collisionSeparationDeterminate = collisionSeparationDeterminate1 + collisionSeparationDeterminate2;

                    currentPerimeterEdge.tectonicPressure = tectonicPressureVector.magnitude * Mathf.Sign(collisionSeparationDeterminate);
                }
            }



            // First pass on the perimeter tiles, just calculate average tectonic pressure and sheer on each perimeter tile
            for (int j = 0; j < currentTectonicPlate.perimeterTiles.Count; j++)
            {
                GlobeTile currentPerimeterTile = currentTectonicPlate.perimeterTiles[j];
                GlobeTile adjacentPlateTile;
                float currentPerimeterTileTotalTectonicPressure = 0f;
                float currentPerimeterTileTotalTectonicSheer = 0f;
                int totalPerimiterEdgeCount = 0;
                for (int k = 0; k < currentPerimeterTile.edges.Count; k++)
                {
                    GlobeTileEdge currentPerimeterTileEdge = currentPerimeterTile.edges[k];
                    int matchingPerimiterTileIndex = currentPerimeterTileEdge.adjacentTiles.IndexOf(currentPerimeterTile);
                    if (matchingPerimiterTileIndex == 0)
                    {
                        adjacentPlateTile = currentPerimeterTileEdge.adjacentTiles[1];
                    } else
                    {
                        adjacentPlateTile = currentPerimeterTileEdge.adjacentTiles[0];
                    }
                    if (currentTectonicPlate.perimeterEdges.Any(edge => edge.id == currentPerimeterTileEdge.id))
                    {
                        currentPerimeterTileTotalTectonicPressure += currentPerimeterTileEdge.tectonicPressure;
                        currentPerimeterTileTotalTectonicSheer += currentPerimeterTileEdge.tectonicSheer;
                        totalPerimiterEdgeCount++;
                    }
                }
                // For each perimeter tile, the tectonic pressure and sheer are averaged among all the plate perimeter edges on that tile
                float averagePerimeterTileTectonicPressure = currentPerimeterTileTotalTectonicPressure / totalPerimiterEdgeCount;
                float averagePerimeterTileTectonicSheer = currentPerimeterTileTotalTectonicSheer / totalPerimiterEdgeCount;
                currentPerimeterTile.tectonicPressure = averagePerimeterTileTectonicPressure;
                currentPerimeterTile.tectonicSheer = averagePerimeterTileTectonicSheer;

            }


            // Second pass on the perimeter tiles, modify each perimeter tile's tectonic pressure and sheer based on what types of plates are interacting
            //for (int j = 0; j < currentTectonicPlate.perimeterTiles.Count; j++)
            //{
            //    GlobeTile currentPerimeterTile = currentTectonicPlate.perimeterTiles[j];
            //    GlobeTile adjacentPlateTile;
            //    for (int k = 0; k < currentPerimeterTile.edges.Count; k++)
            //    {
            //        GlobeTileEdge currentPerimeterTileEdge = currentPerimeterTile.edges[k];
            //        if (currentTectonicPlate.perimeterEdges.Any(edge => edge.id == currentPerimeterTileEdge.id))
            //        {
            //            int matchingPerimiterTileIndex = currentPerimeterTileEdge.adjacentTiles.IndexOf(currentPerimeterTile);
            //            if (matchingPerimiterTileIndex == 0)
            //            {
            //                adjacentPlateTile = currentPerimeterTileEdge.adjacentTiles[1];
            //            }
            //            else
            //            {
            //                adjacentPlateTile = currentPerimeterTileEdge.adjacentTiles[0];
            //            }

            //            if (currentTectonicPlate.plateType == TectonicPlate.CONTINENTAL && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.CONTINENTAL)
            //            {
            //                currentPerimeterTile.tectonicPressure += Mathf.Sign(currentPerimeterTileEdge.tectonicPressure) * 0.1f;
            //            }
            //            else if (currentTectonicPlate.plateType == TectonicPlate.CONTINENTAL && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.OCEANIC)
            //            {
            //                currentPerimeterTile.tectonicPressure += Mathf.Sign(currentPerimeterTileEdge.tectonicPressure) * 0.2f;
            //            }
            //            else if (currentTectonicPlate.plateType == TectonicPlate.OCEANIC && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.CONTINENTAL)
            //            {
            //                currentPerimeterTile.tectonicPressure -= Mathf.Sign(currentPerimeterTileEdge.tectonicPressure) * 0.2f;
            //            }
            //            else if (currentTectonicPlate.plateType == TectonicPlate.OCEANIC && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.OCEANIC)
            //            {
            //                currentPerimeterTile.tectonicPressure += Mathf.Sign(currentPerimeterTileEdge.tectonicPressure) * 0.05f;
            //            }
            //        }
            //    }
            //}

            // Get the Average Tectonic Pressure and Sheer between all the perimeter tiles on this plate
            float totalPerimeterTectonicPressure = 0f;
            float totalPerimeterTectonicSheer = 0f;
            for (int j = 0; j < currentTectonicPlate.perimeterTiles.Count; j++)
            {
                totalPerimeterTectonicPressure += currentTectonicPlate.perimeterTiles[j].tectonicPressure;
                totalPerimeterTectonicSheer += currentTectonicPlate.perimeterTiles[j].tectonicSheer;
            }
            float averagePerimeterTectonicPressure = totalPerimeterTectonicPressure / currentTectonicPlate.perimeterTiles.Count;
            float averagePerimeterTectonicSheer = totalPerimeterTectonicSheer / currentTectonicPlate.perimeterTiles.Count;

            // Interpolate the tectonic pressure and sheer of the non-perimeter tiles of the tectonic plate against the average pressure and sheer for the whole perimeter
            for (int j = 0; j < currentTectonicPlate.plateTiles.Count; j++)
            {
                GlobeTile currentPlateTile = currentTectonicPlate.plateTiles[j];
                if (currentTectonicPlate.perimeterTiles.Any(tile => tile.id == currentPlateTile.id)) 
                {
                    continue;
                } 
                else
                {
                    currentPlateTile.tectonicPressure = Mathf.Lerp(averagePerimeterTectonicPressure, currentPlateTile.closestTectonicPerimiterTile.tectonicPressure, 1f / currentPlateTile.tilesAwayFromPlatePerimeter);
                    currentPlateTile.tectonicSheer = Mathf.Lerp(averagePerimeterTectonicSheer, currentPlateTile.closestTectonicPerimiterTile.tectonicSheer, 1f / currentPlateTile.tilesAwayFromPlatePerimeter);
                }
            }

        }

        // Normalize tectonic pressure and sheer so that pressure is in [-1, 1] and sheer is in [0, 1] across the globe
        List<GlobeTile> tilesSortedByTectonicPressure = new List<GlobeTile>(globeTiles);
        List<GlobeTile> tilesSortedByTectonicSheer = new List<GlobeTile>(globeTiles);
        tilesSortedByTectonicPressure.Sort((a, b) => b.tectonicPressure.CompareTo(a.tectonicPressure));
        tilesSortedByTectonicSheer.Sort((a, b) => b.tectonicSheer.CompareTo(a.tectonicSheer));
        float greatestTectonicPressure = tilesSortedByTectonicPressure[0].tectonicPressure;
        float greatestTectonicSheer = tilesSortedByTectonicSheer[0].tectonicSheer;
        float leastTectonicPressure = tilesSortedByTectonicPressure[tilesSortedByTectonicPressure.Count - 1].tectonicPressure;
        float leastTectonicSheer = tilesSortedByTectonicSheer[tilesSortedByTectonicSheer.Count - 1].tectonicSheer;
        for (int i = 0; i < globeTiles.Count; i++)
        {
            GlobeTile currentPlateTile = globeTiles[i];
            currentPlateTile.tectonicPressure = (2 * ((currentPlateTile.tectonicPressure - leastTectonicPressure) / (greatestTectonicPressure - leastTectonicPressure))) - 1;
            currentPlateTile.tectonicSheer = (currentPlateTile.tectonicSheer - leastTectonicSheer) / (greatestTectonicSheer - leastTectonicSheer);
        }

        // Determine the elevations for the tiles of each tectonic plate
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            // First we need to determine the elevation of each tile on the perimeter of the tectonic plate by looking at how it's interacting with adjacent plates
            TectonicPlate currentTectonicPlate = tectonicPlates[i];
            for (int j = 0; j < currentTectonicPlate.perimeterTiles.Count; j++)
            {
                GlobeTile currentPerimeterTile = currentTectonicPlate.perimeterTiles[j];
                GlobeTile adjacentPlateTile;
                for (int k = 0; k < currentPerimeterTile.edges.Count; k++)
                {
                    GlobeTileEdge currentPerimeterTileEdge = currentPerimeterTile.edges[k];
                    if (currentTectonicPlate.perimeterEdges.Any(edge => edge.id == currentPerimeterTileEdge.id))
                    {
                        int matchingPerimiterTileIndex = currentPerimeterTileEdge.adjacentTiles.IndexOf(currentPerimeterTile);
                        if (matchingPerimiterTileIndex == 0)
                        {
                            adjacentPlateTile = currentPerimeterTileEdge.adjacentTiles[1];
                        }
                        else
                        {
                            adjacentPlateTile = currentPerimeterTileEdge.adjacentTiles[0];
                        }

                        if (currentTectonicPlate.plateType == TectonicPlate.CONTINENTAL && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.CONTINENTAL)
                        {
                            if (currentPerimeterTile.tectonicPressure > 0)
                            {
                                // (current plate) CONTINENTAL --> | <-- CONTINENTAL (adjacent plate)
                                float alterElevation = Random.value;
                                if (alterElevation > 0.4f)
                                {
                                    currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure;
                                }
                            } else {
                                // (current plate) CONTINENTAL <-- | --> CONTINENTAL (adjacent plate)
                                float alterElevation = Random.value;
                                if (alterElevation > 0.8f)
                                {
                                    currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure;
                                }
                            }
                        }
                        else if (currentTectonicPlate.plateType == TectonicPlate.CONTINENTAL && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.OCEANIC)
                        {
                            if (currentPerimeterTile.tectonicPressure > 0)
                            {
                                // (current plate) CONTINENTAL --> | <-- OCEANIC (adjacent plate)
                                float alterElevation = Random.value;
                                if (alterElevation > 0.4f)
                                {
                                    currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure;
                                }
                            } else {
                                // (current plate) CONTINENTAL <-- | --> OCEANIC (adjacent plate)
                                currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure / 2.2f;
                            }
                        }
                        else if (currentTectonicPlate.plateType == TectonicPlate.OCEANIC && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.CONTINENTAL)
                        {
                            if (currentPerimeterTile.tectonicPressure > 0)
                            {
                                // (current plate) OCEANIC --> | <-- CONTINENTAL (adjacent plate)
                                currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure;
                            } else {
                                // (current plate) OCEANIC <-- | --> CONTINENTAL (adjacent plate)
                                //currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure / 2.2f;
                            }
                        }
                        else if (currentTectonicPlate.plateType == TectonicPlate.OCEANIC && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.OCEANIC)
                        {
                            if (currentPerimeterTile.tectonicPressure > 0)
                            {
                                // (current plate) OCEANIC --> | <-- OCEANIC (adjacent plate)
                                float alterElevation = Random.value;
                                if (alterElevation > 0.8f)
                                {
                                    currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure;
                                }
                            } else {
                                // (current plate) OCEANIC <-- | --> OCEANIC (adjacent plate)
                                currentPerimeterTile.elevation += currentPerimeterTile.tectonicPressure / 2.2f;
                            }
                        }


                        if (currentPerimeterTile.elevation > 1f)
                        {
                            currentPerimeterTile.elevation = 1f;
                        }
                        else if (currentPerimeterTile.elevation < -1f)
                        {
                            currentPerimeterTile.elevation = -1f;
                        }
                    }
                }
                if (currentPerimeterTile.elevation <= 0f)
                {
                    currentPerimeterTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterMaterial;
                }
                else
                {
                    currentPerimeterTile.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
                }
            }

            // Interpolate the elevation of non-perimeter tiles of the tectonic plate
            for (int j = 0; j < currentTectonicPlate.plateTiles.Count; j++)
            {
                GlobeTile currentPlateTile = currentTectonicPlate.plateTiles[j];
                if (currentTectonicPlate.perimeterTiles.Any(tile => tile.id == currentPlateTile.id))
                {
                    continue;
                }
                else
                {
                    currentPlateTile.elevation = Mathf.Lerp(currentPlateTile.elevation, currentPlateTile.closestTectonicPerimiterTile.elevation, 1f / currentPlateTile.tilesAwayFromPlatePerimeter);
                }
                if (currentPlateTile.elevation <= 0f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterMaterial;
                }
                else
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
                }
            }
        }







        // Debugging Tectonic Plates
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            //tectonicPlates[i].seed.terrain.GetComponent<MeshRenderer>().material = (tectonicPlates[i].plateType == TectonicPlate.CONTINENTAL ? Globe.GrasslandMaterial : Globe.SnowMaterial);
            //print(tectonicPlates[i].plateTiles.Count);
            //print(tectonicPlates[i].perimeterTiles.Count);
            //print(tectonicPlates[i].perimeterTileNeighbors.Count);
            //print(tectonicPlates[i].perimeterEdges.Count);
            //tectonicPlates[i].seed.terrain.GetComponent<MeshRenderer>().material = Globe.GrasslandMaterial;
            //tectonicPlates[i].plateTiles.Sort((a, b) => a.tilesAwayFromPlatePerimeter.CompareTo(b.tilesAwayFromPlatePerimeter));
            for (int j = 0; j < tectonicPlates[i].plateTiles.Count; j++)
            {
                GlobeTile currentPlateTile = tectonicPlates[i].plateTiles[j];
                //print(tectonicPlates[i].plateTiles[j].tilesAwayFromPlatePerimeter);
                //tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
                //switch (tectonicPlates[i].plateTiles[j].tilesAwayFromPlatePerimeter)
                //{
                //    case 0:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(0 / 8f, 0 / 8f, 0 / 8f);
                //        break;
                //    case 1:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(1 / 8f, 1 / 8f, 1 / 8f);
                //        break;
                //    case 2:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(2 / 8f, 2 / 8f, 2 / 8f);
                //        break;
                //    case 3:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(3 / 8f, 3 / 8f, 3 / 8f);
                //        break;
                //    case 4:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(4 / 8f, 4 / 8f, 4 / 8f);
                //        break;
                //    case 5:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(5 / 8f, 5 / 8f, 5 / 8f);
                //        break;
                //    case 6:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(6 / 8f, 6 / 8f, 6 / 8f);
                //        break;
                //    case 7:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(7 / 8f, 7 / 8f, 7 / 8f);
                //        break;
                //    case 8:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(8 / 8f, 8 / 8f, 8 / 8f);
                //        break;
                //    default:
                //        tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material.color = new Color(9 / 8f, 9 / 8f, 9 / 8f);
                //        break;
                //}

                if (currentPlateTile.elevation > 0.9f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation10Material;
                } else if (currentPlateTile.elevation <= 0.9f && currentPlateTile.elevation > 0.8f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation9Material;
                }
                else if (currentPlateTile.elevation <= 0.8f && currentPlateTile.elevation > 0.7f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation8Material;
                }
                else if (currentPlateTile.elevation <= 0.7f && currentPlateTile.elevation > 0.6f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation7Material;
                }
                else if (currentPlateTile.elevation <= 0.6f && currentPlateTile.elevation > 0.5f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation6Material;
                }
                else if (currentPlateTile.elevation <= 0.5f && currentPlateTile.elevation > 0.4f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation5Material;
                }
                else if (currentPlateTile.elevation <= 0.4f && currentPlateTile.elevation > 0.3f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation4Material;
                }
                else if (currentPlateTile.elevation <= 0.3f && currentPlateTile.elevation > 0.2f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation3Material;
                }
                else if (currentPlateTile.elevation <= 0.2f && currentPlateTile.elevation > 0.1f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation2Material;
                }
                else if (currentPlateTile.elevation <= 0.1f && currentPlateTile.elevation > 0.0f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LandElevation1Material;
                }
                else if (currentPlateTile.elevation <= 0.0f && currentPlateTile.elevation > -0.1f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation1Material;
                }
                else if (currentPlateTile.elevation <= -0.1f && currentPlateTile.elevation > -0.2f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation2Material;
                }
                else if (currentPlateTile.elevation <= -0.2f && currentPlateTile.elevation > -0.3f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation3Material;
                }
                else if (currentPlateTile.elevation <= -0.3f && currentPlateTile.elevation > -0.4f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation4Material;
                }
                else if (currentPlateTile.elevation <= -0.4f && currentPlateTile.elevation > -0.5f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation5Material;
                }
                else if (currentPlateTile.elevation <= -0.5f && currentPlateTile.elevation > -0.6f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation6Material;
                }
                else if (currentPlateTile.elevation <= -0.6f && currentPlateTile.elevation > -0.7f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation7Material;
                }
                else if (currentPlateTile.elevation <= -0.7f && currentPlateTile.elevation > -0.8f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation4Material;
                }
                else if (currentPlateTile.elevation <= -0.8f && currentPlateTile.elevation > -0.9f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation9Material;
                }
                else if (currentPlateTile.elevation <= -0.9f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation10Material;
                }
            }
        }


        // Visualise tile motion vectors
        for (int i = 0; i < globeTiles.Count; i++)
        {
            GameObject globeTileMotionVectors = GameObject.Find("GlobeTileMotionVectors");
            LineRenderer line = new GameObject().AddComponent<LineRenderer>();
            line.transform.parent = globeTileMotionVectors.transform;

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
            GameObject globeTileGrid = GameObject.Find("GlobeTileGrid");
            LineRenderer line = new GameObject().AddComponent<LineRenderer>();
            line.transform.parent = globeTileGrid.transform;
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

        // Visualize tectonic plate boundaries by their tectonic pressure
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            TectonicPlate currentPlate = tectonicPlates[i];
            for (int j = 0; j < currentPlate.perimeterEdges.Count; j++)
            {
                GlobeTileEdge currentPlateTileEdge = currentPlate.perimeterEdges[j];
                GameObject tectonicForceLines = GameObject.Find("TectonicForceLines");
                LineRenderer line = new GameObject().AddComponent<LineRenderer>();
                line.transform.parent = tectonicForceLines.transform;
                line.material = Globe.EdgeMaterial;

                if (currentPlateTileEdge.tectonicPressure <= -1f)
                {
                    line.material.color = new Color(0f, 255f, 0f);
                }
                else if (currentPlateTileEdge.tectonicPressure > -1f && currentPlateTileEdge.tectonicPressure <= -0.5f)
                {
                    line.material.color = new Color(1.5f, 255f, 0f);
                }
                else if (currentPlateTileEdge.tectonicPressure > -0.5f && currentPlateTileEdge.tectonicPressure <= 0.5f)
                {
                    line.material.color = new Color(255f, 255f, 0f);
                }
                else if (currentPlateTileEdge.tectonicPressure > 0.5f && currentPlateTileEdge.tectonicPressure <= 1f)
                {
                    line.material.color = new Color(255f, 1.5f, 0f);
                }
                else if (currentPlateTileEdge.tectonicPressure > 1f)
                {
                    line.material.color = new Color(255f, 0f, 0f);
                }


                line.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

                line.startWidth = 0.15f;
                line.endWidth = 0.15f;
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
