using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Diagnostics;

public class Globe : MonoBehaviour
{
    private struct UnitSpherePoint
    {
        public static List<Vector3> AllVectors = new List<Vector3>();
        public static List<UnitSpherePoint> AllUnitSpherePoints = new List<UnitSpherePoint>();
        public Vector3 point { get; }
        public float phi { get; }
        public float theta { get; }
        public UnitSpherePoint(Vector3 point, float phi, float theta)
        {
            this.point = point;
            this.phi = phi;
            this.theta = theta;
            AllVectors.Add(this.point);
            AllUnitSpherePoints.Add(this);
        }
    }
    private class MoistureContributor
    {
        public static List<List<MoistureContributor>> AllMoistureContributorLists = new List<List<MoistureContributor>>();
        public AtmospherePoint recipientPoint { get; }
        public AtmospherePoint contributorAtmospherePoint { get; }
        public int possibleNumberOfContributors { get; set; }
        public float delaunayWindDotProduct { get; }
        public float windDirectionDotProduct { get; }
        public float relativeElevation { get; }
        public float contributorMoisture { get; }
        public MoistureContributor(AtmospherePoint recipientPoint, AtmospherePoint contributorAtmospherePoint, int possibleNumberOfContributors, float delaunayWindDotProduct, float windDirectionDotProduct, float relativeElevation, float contributorMoisture)
        {
            this.recipientPoint = recipientPoint;
            this.contributorAtmospherePoint = contributorAtmospherePoint;
            this.possibleNumberOfContributors = possibleNumberOfContributors;
            this.delaunayWindDotProduct = delaunayWindDotProduct;
            this.windDirectionDotProduct = windDirectionDotProduct;
            this.relativeElevation = relativeElevation;
            this.contributorMoisture = contributorMoisture;
        }
    }

    public int numberOfTiles = 500;
    public float scale = 20;
    public float jitter = 0;
    public int numberOfTectonicPlates = 15;
    public int tectonicPlateSmoothness = 0;
    public int oceanicRate = 70;
    public float globeAverageTemperature = 273.15f;
    public float temperatureVariability = 50f;
    public int moistureIterations = 5;
    private List<UnitSpherePoint> tileGraphNodes;
    public static List<GlobeTile> globeTiles = new List<GlobeTile>();
    private List<TectonicPlate> globePlates = new List<TectonicPlate>();
    private bool globeGenerationFinished = false;
    public Atmosphere atmosphere = new Atmosphere();

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

    public static Material LT241_5KMaterial;
    public static Material LT249_5KMaterial;
    public static Material LT257_5KMaterial;
    public static Material LT265_5KMaterial;
    public static Material LT273_5KMaterial;
    public static Material LT281_5KMaterial;
    public static Material LT289_5KMaterial;
    public static Material LT297_5KMaterial;
    public static Material LT305_5KMaterial;
    public static Material LT313_5KMaterial;


    // Start is called before the first frame update
    void Start()
    {
        Stopwatch overallPerformance = Stopwatch.StartNew();
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

        LT241_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[0]);
        LT249_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[1]);
        LT257_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[2]);
        LT265_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[3]);
        LT273_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[4]);
        LT281_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[5]);
        LT289_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[6]);
        LT297_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[7]);
        LT305_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[8]);
        LT313_5KMaterial = new Material(GameObject.Find("TemperatureMaterials").GetComponent<MeshRenderer>().materials[9]);

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
        Stopwatch delaunayConvexHullGeneration = Stopwatch.StartNew();
        convexHullCalculator.GenerateHull(UnitSpherePoint.AllVectors, true, ref delaunayVerticies, ref delaunayTriangles, ref delaunayNormals);
        delaunayConvexHullGeneration.Stop();
        print("Delaunay Convex Hull Generation -> " + delaunayConvexHullGeneration.Elapsed);

        // Determine centroids of each delaunay triangle on the convex hull
        Stopwatch centroidDetermination = Stopwatch.StartNew();
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
        centroidDetermination.Stop();
        print("Centroid Determination -> " + centroidDetermination.Elapsed);

        // We don't need this anymore
        // Generate the centroidal voronoi convex hull
        //convexHullCalculator.GenerateHull(CentroidPoint.All, true, ref centroidVerticies, ref centroidTriangles, ref centroidNormals);

        //Determine each tile (First Attempt)
        //Stopwatch tileDetermination = Stopwatch.StartNew();
        //List<Vector3> tileVertices = new List<Vector3>();
        //for (int i = 0; i < centroids.Count; i++)
        //{
        //    CentroidPoint currentCentroidPoint = centroids[i];
        //    for (int p = 0; p < currentCentroidPoint.delaunayParents.Count; p++)
        //    {
        //        Vector3 currentCentroidPointParent = currentCentroidPoint.delaunayParents[p];
        //        for (int j = 0; j < centroids.Count; j++)
        //        {
        //            CentroidPoint nextCentroidPoint = centroids[j];
        //            for (int k = 0; k < nextCentroidPoint.delaunayParents.Count; k++)
        //            {
        //                Vector3 nextCentroidPointCurrentDelaunayParent = nextCentroidPoint.delaunayParents[k];
        //                if (currentCentroidPointParent == nextCentroidPointCurrentDelaunayParent)
        //                {
        //                    tileVertices.Add(nextCentroidPoint.centroid);
        //                }
        //            }
        //        }
        //        if (globeTiles.Find(tile => tile.delaunayPoint == currentCentroidPointParent) == null)
        //        {
        //            Vector3 globeTileVector = new Vector3(currentCentroidPointParent.x, currentCentroidPointParent.y, currentCentroidPointParent.z);
        //            Vector3 globeTileMatchingUnitSpherePoint = UnitSpherePoint.AllVectors.Find(point => point.Equals(globeTileVector));
        //            globeTiles.Add(new GlobeTile(globeTileVector, new List<Vector3>(tileVertices), scale, UnitSpherePoint.AllUnitSpherePoints[UnitSpherePoint.AllVectors.IndexOf(globeTileVector)].theta));
        //        }
        //        tileVertices.Clear();
        //    }
        //}
        //tileDetermination.Stop();
        //print("Tile Determination -> " + tileDetermination.Elapsed);

        //Determine each tile (Second Attempt)
        Stopwatch tileDetermination = Stopwatch.StartNew();
        List<Vector3> tileVertices = new List<Vector3>();
        for (int i = 0; i < centroids.Count; i++)
        {
            CentroidPoint currentCentroidPoint = centroids[i];
            for (int p = 0; p < currentCentroidPoint.delaunayParents.Count; p++)
            {
                Vector3 currentCentroidPointParent = currentCentroidPoint.delaunayParents[p];
                List<CentroidPoint> centroidsSharingCurrentDelaunayParent = centroids.FindAll(point => point.delaunayParents.Contains(currentCentroidPointParent));
                for (int j = 0; j < centroidsSharingCurrentDelaunayParent.Count; j++)
                {
                    CentroidPoint nextCentroidPoint = centroidsSharingCurrentDelaunayParent[j];
                    tileVertices.Add(nextCentroidPoint.centroid);
                }
                if (globeTiles.Find(tile => tile.delaunayPoint == currentCentroidPointParent) == null)
                {
                    Vector3 globeTileVector = new Vector3(currentCentroidPointParent.x, currentCentroidPointParent.y, currentCentroidPointParent.z);
                    Vector3 globeTileMatchingUnitSpherePoint = UnitSpherePoint.AllVectors.Find(point => point.Equals(globeTileVector));
                    GlobeTile nextGlobeTile = new GlobeTile(globeTileVector,
                        new List<Vector3>(tileVertices),
                        scale,
                        UnitSpherePoint.AllUnitSpherePoints[UnitSpherePoint.AllVectors.IndexOf(globeTileVector)].phi,
                        UnitSpherePoint.AllUnitSpherePoints[UnitSpherePoint.AllVectors.IndexOf(globeTileVector)].theta,
                        this.globeAverageTemperature,
                        this.temperatureVariability
                    );
                    AtmospherePoint nextAtmospherePoint = new AtmospherePoint(nextGlobeTile, moistureIterations);
                    nextGlobeTile.atmospherePoint = nextAtmospherePoint;
                    globeTiles.Add(nextGlobeTile);
                    atmosphere.atmospherePoints.Add(nextAtmospherePoint);
                }
                tileVertices.Clear();
            }
        }
        tileDetermination.Stop();
        print("Tile Determination -> " + tileDetermination.Elapsed);


        // Determine each tile's neighbors and edges
        Stopwatch neighborEdgeDetermination = Stopwatch.StartNew();
        List<GlobeTile> neighboringTiles = new List<GlobeTile>();
        globeTiles = new List<GlobeTile>(globeTiles.OrderBy(tile => Vector3.Distance(globeTiles[0].delaunayPoint, tile.delaunayPoint)));
        for (int i = 0; i < globeTiles.Count; i++)
        {
            GlobeTile currentGlobeTile = globeTiles[i];
            // Attempted optimization, actually slows this process down!
            //neighboringTiles = globeTiles.FindAll(tile => tile.vertices.Any(match => currentGlobeTile.vertices.Contains(match)));
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
                currentGlobeTile.atmospherePoint.neighborPoints.Add(neighboringTiles[j].atmospherePoint);
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
        neighborEdgeDetermination.Stop();
        print("Neighbor and Edge Determination -> " + neighborEdgeDetermination.Elapsed);

        // Determine each atmosphere point's downwind neighbors
        Stopwatch atmosphereDownwindNeighborDetermination = Stopwatch.StartNew();
        for (int i = 0; i < atmosphere.atmospherePoints.Count; i++)
        {
            AtmospherePoint currentAtmospherePoint = atmosphere.atmospherePoints[i];
            SortedDictionary<float, AtmospherePoint> dotProductPointPair = new SortedDictionary<float, AtmospherePoint>();

            for (int j = 0; j < currentAtmospherePoint.neighborPoints.Count; j++)
            {
                AtmospherePoint currentNeighborAtmospherePoint = currentAtmospherePoint.neighborPoints[j];
                float delaunayWindDotProduct = Vector3.Dot(currentNeighborAtmospherePoint.delaunayPoint - currentAtmospherePoint.delaunayPoint, currentAtmospherePoint.windDirection);
                if (delaunayWindDotProduct > 0)
                {
                    dotProductPointPair.Add(delaunayWindDotProduct, currentNeighborAtmospherePoint);
                    currentAtmospherePoint.totalDownwindFlow += delaunayWindDotProduct;
                }
            }
            foreach (KeyValuePair<float, AtmospherePoint> pair in dotProductPointPair)
            {
                currentAtmospherePoint.downwindNeighbors.Add(pair.Key / currentAtmospherePoint.totalDownwindFlow, pair.Value);
            }
        }
        
        atmosphereDownwindNeighborDetermination.Stop();
        print("Atmosphere Downwind Neighbor Point Determination -> " + atmosphereDownwindNeighborDetermination.Elapsed);





        // Generate Tectonic Plate Seeds
        Stopwatch tectonicSeedGeneration = Stopwatch.StartNew();
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
        tectonicSeedGeneration.Stop();
        print("Tectonic Seed Generation -> " + tectonicSeedGeneration.Elapsed);

        // Build Tectonic Plates
        List<TectonicPlate> tectonicPlates = new List<TectonicPlate>();
        for (int i = 0; i < tectonicPlateSeeds.Count; i++)
        {
            tectonicPlates.Add(new TectonicPlate(tectonicPlateSeeds[i], tectonicPlateSmoothness, oceanicRate));
        }

        // Working First Attempt (Fairly slow)
        Stopwatch tectonicPlateBuilding = Stopwatch.StartNew();
        Stopwatch findNextPlateTotalTime = Stopwatch.StartNew();
        findNextPlateTotalTime.Stop();
        findNextPlateTotalTime.Reset();
        List<int> finishedPlateIndices = new List<int>();
        while (finishedPlateIndices.Count < tectonicPlates.Count)
        {
            for (int i = 0; i < tectonicPlates.Count; i++)
            {
                if (!finishedPlateIndices.Contains(i))
                {
                    findNextPlateTotalTime = Stopwatch.StartNew();
                    if (!tectonicPlates[i].FindNextTiles())
                    {
                        finishedPlateIndices.Add(i);
                    }
                    findNextPlateTotalTime.Stop();
                }
            }
        }
        tectonicPlateBuilding.Stop();
        print("Tectonic Plate Building -> " + tectonicPlateBuilding.Elapsed);
        print("Find Next Plate Total -> " + findNextPlateTotalTime.Elapsed);

        // Determine the stresses on each edge of each tectonic plate
        Stopwatch tectonicForceDetermination = Stopwatch.StartNew();
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
        tectonicForceDetermination.Stop();
        print("Tectonic Force Determination -> " + tectonicForceDetermination.Elapsed);

        // Normalize tectonic pressure and sheer so that pressure is in [-1, 1] and sheer is in [0, 1] across the globe
        Stopwatch tectonicForceNormalization = Stopwatch.StartNew();
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
        tectonicForceNormalization.Stop();
        print("Tectonic Force Normalization -> " + tectonicForceNormalization.Elapsed);


        // Determine the elevations for the tiles of each tectonic plate
        Stopwatch elevationDetermination = Stopwatch.StartNew();
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            // First we need to determine the elevation of each tile on the perimeter of the tectonic plate by looking at how it's interacting with adjacent plates
            TectonicPlate currentTectonicPlate = tectonicPlates[i];
            for (int j = 0; j < currentTectonicPlate.perimeterTiles.Count; j++)
            {
                GlobeTile currentPerimeterTile = currentTectonicPlate.perimeterTiles[j];
                GlobeTile adjacentPlateTile;
                int numberOfContributingTiles = 0;
                for (int k = 0; k < currentPerimeterTile.edges.Count; k++)
                {
                    GlobeTileEdge currentPerimeterTileEdge = currentPerimeterTile.edges[k];
                    if (currentTectonicPlate.perimeterEdges.Any(edge => edge.id == currentPerimeterTileEdge.id))
                    {
                        numberOfContributingTiles++;
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
                                //if (alterElevation > 0.4f)
                                //{
                                //    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure * 1.4f));
                                //} else {
                                //    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure * 0.7f));
                                //}
                                currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure * 1.4f));
                            } else {
                                // (current plate) CONTINENTAL <-- | --> CONTINENTAL (adjacent plate)
                                float alterElevation = Random.value;
                                if (alterElevation > 0.2f)
                                {
                                    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation - (currentPerimeterTile.tectonicPressure / 10f));
                                } else {
                                    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure / 10f));
                                }
                            }
                        }
                        else if (currentTectonicPlate.plateType == TectonicPlate.CONTINENTAL && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.OCEANIC)
                        {
                            if (currentPerimeterTile.tectonicPressure > 0)
                            {
                                // (current plate) CONTINENTAL --> | <-- OCEANIC (adjacent plate)
                                float alterElevation = Random.value;
                                //if (alterElevation > 0.4f)
                                //{
                                //    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure * 1.4f));
                                //} else
                                //{
                                //    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure * 0.7f));
                                //}
                                currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure * 1.4f));
                            } else {
                                // (current plate) CONTINENTAL <-- | --> OCEANIC (adjacent plate)
                                // non-normalized
                                currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure / 6f));
                                // normalized
                                //currentPerimeterTile.SetElevation(currentPerimeterTile.elevation - (currentPerimeterTile.tectonicPressure / 10f));
                            }
                        }
                        else if (currentTectonicPlate.plateType == TectonicPlate.OCEANIC && adjacentPlateTile.tectonicPlate.plateType == TectonicPlate.CONTINENTAL)
                        {
                            if (currentPerimeterTile.tectonicPressure > 0)
                            {
                                // (current plate) OCEANIC --> | <-- CONTINENTAL (adjacent plate)
                                currentPerimeterTile.SetElevation(currentPerimeterTile.elevation - currentPerimeterTile.tectonicPressure);
                            } else {
                                // (current plate) OCEANIC <-- | --> CONTINENTAL (adjacent plate)
                                currentPerimeterTile.SetElevation(currentPerimeterTile.elevation - (currentPerimeterTile.tectonicPressure / 2.5f));
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
                                    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + currentPerimeterTile.tectonicPressure);
                                }
                            } else {
                                // (current plate) OCEANIC <-- | --> OCEANIC (adjacent plate)
                                currentPerimeterTile.SetElevation(currentPerimeterTile.elevation + (currentPerimeterTile.tectonicPressure / 2.2f));
                            }
                        }
                    }
                }
                if (numberOfContributingTiles > 0)
                {
                    currentPerimeterTile.SetElevation(currentPerimeterTile.elevation / numberOfContributingTiles);
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
                    currentPlateTile.SetElevation(Mathf.Lerp(currentPlateTile.elevation, currentPlateTile.closestTectonicPerimiterTile.elevation, 1f / currentPlateTile.tilesAwayFromPlatePerimeter));
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
        elevationDetermination.Stop();
        print("Elevation Determination -> " + elevationDetermination.Elapsed);

        // Normalize elevation after all interpolation is done
        Stopwatch elevationNormalization = Stopwatch.StartNew();
        for (int i = 0; i < globeTiles.Count; i++)
        {
            GlobeTile currentPlateTile = globeTiles[i];
            currentPlateTile.SetElevation(currentPlateTile.elevation / 1.6f); // 1.6f is the maximum deviation from sea level for elevation.
        }
        elevationNormalization.Stop();
        print("Elevation Normalization -> " + elevationNormalization.Elapsed);

        // Update Temperature and Pressure of each tile based on the elevation of the tile
        for (int i = 0; i < globeTiles.Count; i++)
        {
            GlobeTile currentGlobeTile = globeTiles[i];
            if (currentGlobeTile.terrainType == GlobeTile.LAND)
            {
                currentGlobeTile.surfaceTemperature -= currentGlobeTile.elevation * 45f; // 45f degrees K per unit elevation
                currentGlobeTile.surfaceAirPressure -= currentGlobeTile.elevation * 0.66f; // 0.66f atm per unit elevation
            }
        }


        // Use an iterative process to distribute moisture across the atmosphere, assign to respective tiles
        Stopwatch moistureDistribution = Stopwatch.StartNew();

        /* Moisture Distribution Iteration
         * 
         * All GlobeTiles start with 0.5f surfaceMoisture.
         * 
         * --Evaporation--
         * 
         * Evaporation depends on the surface temp and pressure of the GlobeTile. This will determine a value (evaporationPercent).
         * The amount of moisture evaporating from each tile (evaporationAmount) is equal to the surfaceMoisture * evaporationPercent. 
         * evaporationPercent is in [0.0025,1].
         * surfaceMoisture is in [0,1].
         * 
         * evaporationAmount = surfaceMoisture * evaporationPercent;
         * evaporationAmount is in [0,1].
         * 
         * This evaporationAmount goes into the corresponding AtmospherePoint's airMoisture. We update the surfaceMoisture by subtracting the evaporationAmount.
         * airMoisture is in [iterations, infinity).
         * 
         * Effect of Temperature on Evaporation
         * There is no evaporation occuring if the GlobeTile surfaceTemperature is below 273.15K.
         * There is 100% evaporation occuring if the GlobeTile surfaceTemperature is at or above 373.15K
         * 
         * temperatureImpact = (surfaceTemperature - 273.15) / 100;
         * temperatureImpact = temperatureImpact < 0.05f ? 0.05f : temperatureImpact;
         * temperatureImpact is in [0.05, 1]
         * 
         * Effect of Pressure on Evaporation
         * surfaceAirPressure is in [0.24, 1.1]
         * pressureImpact = (1 / 0.86f) * surfaceAirPressure;
         * pressureImpact = pressureImpact < 0.05f ? 0.05f : pressureImpact;
         * pressureImpact is in [0.05, 1]
         * !! - IMPORTANT - !! - The 0.86f is the difference between max and min possible pressure. This will change if we alter the possible pressure range.
         * 
         * evaporationPercent = temperatureImpact * pressureImpact
         * evaporationPercent is in [0.0025, 1]
         * 
         * --Permeation--
         * 
         * 80% of the airMoisture amount is incremented to the AtmospherePoint's downwind neighbors based on a weighted percentage. The AtmospherePoint's airMoisture is then 20% of where it started.
         * 
         * --Condensation--
         * 
         * Condensation depends on the surface temp and pressure of the AtmospherePoint's GlobTile. This will determine a value (condensationPercent).
         * Lower surfaceTemperature means HIGHER condensationPercent. Lower surfaceAirPressure means HIGHER condensationPercent. condensationPercent is in [0,1].
         * 
         * The surfaceMoistureCapacity of the GlobeTile is equal to 1 - surfaceMoisture. 
         * surfaceMoistureCapacity = 1 - surfaceMoisture;
         * 
         * The condensationAmount is equal to the surfaceMoistureCapacity * condensationPercent;
         * condensationAmount is in [0,1].
         * 
         * Effect of Temperature on Condensation
         * temperatureImpact = ((surfaceTemperature - 273.15) / 100) * -1;
         * temperatureImpact = temperatureImpact > -0.05f ? -0.05f : temperatureImpact;
         * temperatureImpact is in [-1, -0.05]
         * 
         * Effect of Pressure on Condensation
         * surfaceAirPressure is in [0.24, 1.1]
         * pressureImpact = ((1 / 0.86f) * surfaceAirPressure) * -1;
         * pressureImpact = pressureImpact > -0.05f ? -0.05f : pressureImpact;
         * pressureImpact is in [-1, -0.05]
         * !! - IMPORTANT - !! - The 0.86f is the difference between max and min possible pressure. This will change if we alter the possible pressure range.
         * 
         * condensationPercent = temperatureImpact * pressureImpact
         * condensationPercent is in [0.0025, 1]
         * 
         * --Percipitation--
         * 
         * We simply add the condensationAmount to the GlobeTile's surfaceMoisture. 
         * 
         * surfaceMoisture += condensationAmount;
         * 
         * Then, we subtract the condensationAmount from the AtmospherePoint's airMoisture
         * 
         * airMoisture -= condensationAmount.
         * 
         * --End of Iteration--
         * 
         * This is the end of the iteration, process starts again from the beginning.
         * 
         */

        for (int t = 0; t < 2; t++)
        {
            // Evaporation
            for (int i = 0; i < globeTiles.Count; i++)
            {
                GlobeTile currentGlobeTile = globeTiles[i];
                float temperatureImpact = (currentGlobeTile.surfaceTemperature - 273.15f) / 100f;
                temperatureImpact = temperatureImpact < 0.05f ? 0.05f : temperatureImpact;
                float pressureImpact = (1f / 0.86f) * currentGlobeTile.surfaceAirPressure;
                pressureImpact = pressureImpact < 0.05f ? 0.05f : pressureImpact;
                float surfaceEvaporationCapacity = 1f - currentGlobeTile.surfaceMoisture;
                float evaporationPercent = ((2f * temperatureImpact) + pressureImpact + surfaceEvaporationCapacity) / 4f;

                float evaporationAmount = currentGlobeTile.surfaceMoisture * evaporationPercent;
                currentGlobeTile.atmospherePoint.airMoisture += evaporationAmount;
                currentGlobeTile.surfaceMoisture -= evaporationAmount;
            }

            // Permeation
            for (int i = 0; i < globeTiles.Count; i++)
            {
                GlobeTile currentGlobeTile = globeTiles[i];
                AtmospherePoint currentAtmospherePoint = currentGlobeTile.atmospherePoint;
                float permeatedMoistureAmount = currentAtmospherePoint.airMoisture * 1f; // PAY ATTENTION.

                // Split among all the downwind neighbors
                foreach (KeyValuePair<float, AtmospherePoint> neighbor in currentAtmospherePoint.downwindNeighbors)
                {
                    neighbor.Value.airMoisture += permeatedMoistureAmount * neighbor.Key;
                }

                // Only the most downwind neighbor
                //currentAtmospherePoint.downwindNeighbors.Reverse();
                //AtmospherePoint mostDownwindNeighbor = currentAtmospherePoint.downwindNeighbors.Values.ToList()[0];
                //mostDownwindNeighbor.airMoisture += permeatedMoistureAmount;


                currentAtmospherePoint.airMoisture -= permeatedMoistureAmount;
            }

            // Condensation and Percipitation
            for (int i = 0; i < globeTiles.Count; i++)
            {
                GlobeTile currentGlobeTile = globeTiles[i];
                float temperatureImpact = ((currentGlobeTile.surfaceTemperature - 273.15f) / 100f) * 1f;
                temperatureImpact = temperatureImpact < 0.05f ? 0.05f : temperatureImpact;
                float pressureImpact = ((1f / 0.86f) * currentGlobeTile.surfaceAirPressure) * 1f;
                pressureImpact = pressureImpact < 0.05f ? 0.05f : pressureImpact;
                float condensationPercent = (temperatureImpact + (1f - pressureImpact)) / 2f;
                float surfaceMoistureCapacity = 1f - currentGlobeTile.surfaceMoisture;

                float condensationAmount = surfaceMoistureCapacity * condensationPercent;
                currentGlobeTile.surfaceMoisture += condensationAmount;
                currentGlobeTile.atmospherePoint.airMoisture -= condensationAmount;
            }
        }




        //atmosphere.atmospherePoints[100].globeTile.surfaceMoisture = 1f;
        //foreach (KeyValuePair<float, AtmospherePoint> neighbor in atmosphere.atmospherePoints[100].downwindNeighbors)
        //{
        //    neighbor.Value.globeTile.surfaceMoisture += neighbor.Key;
        //}


        moistureDistribution.Stop();
        print("Moisture Distribution -> " + moistureDistribution.Elapsed);





        // Debugging Tectonic Plates
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            for (int j = 0; j < tectonicPlates[i].plateTiles.Count; j++)
            {
                GlobeTile currentPlateTile = tectonicPlates[i].plateTiles[j];

                if (currentPlateTile.elevation > 0.9f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT313_5KMaterial;
                } else if (currentPlateTile.elevation <= 0.9f && currentPlateTile.elevation > 0.8f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT305_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.8f && currentPlateTile.elevation > 0.7f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT297_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.7f && currentPlateTile.elevation > 0.6f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT289_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.6f && currentPlateTile.elevation > 0.5f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT281_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.5f && currentPlateTile.elevation > 0.4f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT273_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.4f && currentPlateTile.elevation > 0.3f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT265_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.3f && currentPlateTile.elevation > 0.2f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT257_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.2f && currentPlateTile.elevation > 0.1f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT249_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.1f && currentPlateTile.elevation > 0.0f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.LT241_5KMaterial;
                }
                else if (currentPlateTile.elevation <= 0.0f && currentPlateTile.elevation > -0.1f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation10Material;
                }
                else if (currentPlateTile.elevation <= -0.1f && currentPlateTile.elevation > -0.2f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation9Material;
                }
                else if (currentPlateTile.elevation <= -0.2f && currentPlateTile.elevation > -0.3f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation8Material;
                }
                else if (currentPlateTile.elevation <= -0.3f && currentPlateTile.elevation > -0.4f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation7Material;
                }
                else if (currentPlateTile.elevation <= -0.4f && currentPlateTile.elevation > -0.5f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation6Material;
                }
                else if (currentPlateTile.elevation <= -0.5f && currentPlateTile.elevation > -0.6f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation5Material;
                }
                else if (currentPlateTile.elevation <= -0.6f && currentPlateTile.elevation > -0.7f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation4Material;
                }
                else if (currentPlateTile.elevation <= -0.7f && currentPlateTile.elevation > -0.8f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation3Material;
                }
                else if (currentPlateTile.elevation <= -0.8f && currentPlateTile.elevation > -0.9f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation2Material;
                }
                else if (currentPlateTile.elevation <= -0.9f)
                {
                    currentPlateTile.terrain.GetComponent<MeshRenderer>().material = Globe.WaterElevation1Material;
                }
            }
        }

        // Surface Temperature
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            for (int j = 0; j < tectonicPlates[i].plateTiles.Count; j++)
            {
                //GlobeTile currentPlateTile = tectonicPlates[i].plateTiles[j];
                //if (currentPlateTile.terrainType == GlobeTile.WATER)
                //{
                //    continue;
                //}
                //tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT313_5KMaterial;
                //if (currentPlateTile.surfaceTemperature <= 313.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT313_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 305.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT305_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 297.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT297_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 289.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT289_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 281.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT281_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 273.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT273_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 265.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT265_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 257.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT257_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 249.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT249_5KMaterial;
                //}
                //if (currentPlateTile.surfaceTemperature <= 241.5f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT241_5KMaterial;
                //}
            }
        }

        // Air Pressure
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            for (int j = 0; j < tectonicPlates[i].plateTiles.Count; j++)
            {
                //GlobeTile currentPlateTile = tectonicPlates[i].plateTiles[j];
                //if (currentPlateTile.terrainType == GlobeTile.WATER)
                //{
                //    continue;
                //}
                //tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT313_5KMaterial;
                //if (currentPlateTile.surfaceAirPressure <= 0.95f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT305_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.88f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT297_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.81f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT289_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.74f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT281_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.67f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT273_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.6f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT265_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.53f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT257_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.46f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT249_5KMaterial;
                //}
                //if (currentPlateTile.surfaceAirPressure <= 0.39f)
                //{
                //    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT241_5KMaterial;
                //}
            }
        }

        // Moisture
        for (int i = 0; i < tectonicPlates.Count; i++)
        {
            for (int j = 0; j < tectonicPlates[i].plateTiles.Count; j++)
            {
                GlobeTile currentPlateTile = tectonicPlates[i].plateTiles[j];
                if (currentPlateTile.terrainType == GlobeTile.WATER)
                {
                    continue;
                }
                tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT313_5KMaterial;
                if (currentPlateTile.surfaceMoisture <= 0.9f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT305_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.8f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT297_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.7f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT289_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.6f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT281_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.5f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT273_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.4f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT265_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.3f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT257_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.2f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT249_5KMaterial;
                }
                if (currentPlateTile.surfaceMoisture <= 0.1f)
                {
                    tectonicPlates[i].plateTiles[j].terrain.GetComponent<MeshRenderer>().material = Globe.LT241_5KMaterial;
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

        // Visualise atmosphere wind vectors
        for (int i = 0; i < atmosphere.atmospherePoints.Count; i++)
        {
            GameObject globeTileMotionVectors = GameObject.Find("AtmosphereWindVectors");
            LineRenderer line = new GameObject().AddComponent<LineRenderer>();
            line.transform.parent = globeTileMotionVectors.transform;
            line.material = Globe.DesertMaterial;
            line.material.color = atmosphere.atmospherePoints[i].color;
            line.GetComponent<Renderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

            line.startWidth = 0.5f;
            line.endWidth = 0f;
            line.startColor = Color.magenta;
            line.endColor = Color.magenta;
            line.positionCount = 2;

            Vector3 atmospherePointScaled = atmosphere.atmospherePoints[i].point;
            Vector3 atmosphereWindEndPoint = atmospherePointScaled + atmosphere.atmospherePoints[i].windDirection;

            line.SetPosition(0, atmospherePointScaled);
            line.SetPosition(1, atmosphereWindEndPoint);
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

        //// Add spheres to each atmosphere point
        //for (int i = 0; i < atmosphere.atmospherePoints.Count; i++)
        //{
        //    tileGraphNodeSpheres.Add(GameObject.CreatePrimitive(PrimitiveType.Sphere));
        //    tileGraphNodeSpheres[i].transform.parent = transform;
        //    tileGraphNodeSpheres[i].transform.position = atmosphere.atmospherePoints[i].delaunayPoint * atmosphere.atmospherePoints[i].scale;
        //    tileGraphNodeSpheres[i].transform.localScale = new Vector3(0.04f, 0.04f, 0.04f);
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
        overallPerformance.Stop();
        print("Overall Performance -> " + overallPerformance.Elapsed);
    }

    //private float nextActionTime = 0f;
    //public float period = 1f;
    //private int tileCounter = 2999;
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

        for (int i = 0; i < numberOfTiles; i++)
        {
            if (jitter > 100)
            {
                jitter = 100;
            }
            int jitterThetaDirection = Random.value > 0.5f ? 1 : -1;
            float jitterTheta = ((Random.value / (numberOfTiles)) * jitter) * jitterThetaDirection;

            float k = i + .5f;

            float phi = Mathf.Acos(1f - 2f * k / numberOfTiles);
            float theta = Mathf.PI * (1 + Mathf.Sqrt(5)) * k + jitterTheta;

            float x = Mathf.Cos(theta) * Mathf.Sin(phi);
            float y = Mathf.Cos(phi);
            float z = Mathf.Sin(theta) * Mathf.Sin(phi);

            tileGraphNodes.Add(new UnitSpherePoint(new Vector3(x, y, z), phi, theta));
        }


        return tileGraphNodes;
    }
}
