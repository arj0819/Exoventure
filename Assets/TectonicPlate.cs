using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class TectonicPlate
{
    //private struct SuitableNeighborTile
    //{
    //    public SuitableNeighborTile(int adjacencyCount, GlobeTile tile)
    //    {
    //        this.adjacencyCount = adjacencyCount;
    //        this.tile = tile;
    //    }
    //    public int adjacencyCount { get; }
    //    public GlobeTile tile { get; }
    //}
    public static int TotalTectonicPlateCount = 0;
    public const int OCEANIC = 0;
    public const int CONTINENTAL = 1;

    public int id = TotalTectonicPlateCount;
    public int plateType;
    private static List<TectonicPlate> AllPlates = new List<TectonicPlate>();
    private static List<GlobeTile> AllPlateTiles = new List<GlobeTile>();
    public GlobeTile seed;
    public List<GlobeTile> plateTiles = new List<GlobeTile>();
    public List<GlobeTile> perimeterTiles = new List<GlobeTile>();
    public List<GlobeTile> perimeterTileNeighbors = new List<GlobeTile>();
    public List<GlobeTileEdge> perimeterEdges = new List<GlobeTileEdge>();
    private List<GlobeTile> possibleNeighbors = new List<GlobeTile>();
    private int smoothness;
    private Vector3 planetaryRotationAxis = new Vector3(Random.value - 0.5f, Random.value - 0.5f, Random.value - 0.5f).normalized;
    private float planetaryRotationScalar = Random.value + 0.1f;
    private Vector3 plateRotationAxis;
    private float plateRotationScalar = Random.value + 0.1f;
    private Color plateColor = new Color(Random.value, Random.value, Random.value);

    public float elevation;
    public float moisture = Random.value * 100;

    public TectonicPlate(GlobeTile seed, int smoothness, int oceanicRate)
    {
        if (oceanicRate > 100)
        {
            oceanicRate = 100;
        } else if (oceanicRate < 0)
        {
            oceanicRate = 0;
        }
        if (smoothness > 100)
        {
            smoothness = 100;
        }
        else if (smoothness < 0)
        {
            smoothness = 0;
        }

        this.plateType = (Random.value * 100) > oceanicRate ? CONTINENTAL : OCEANIC;
        float possibleElevation = this.plateType == CONTINENTAL ? Random.value : (Random.value) * -1;
        while (possibleElevation == 0f)
        {
            possibleElevation = this.plateType == CONTINENTAL ? Random.value : (Random.value) * -1;
        }
        this.elevation = possibleElevation;

        TotalTectonicPlateCount++;
        AllPlates.Add(this);
        AllPlateTiles.Add(seed);
        this.seed = seed;
        this.seed.tilesAwayFromPlateSeed = 0;
        this.plateRotationAxis = this.seed.delaunayPoint.normalized;
        this.seed.motion = Vector3.Cross(this.seed.delaunayPoint, this.planetaryRotationAxis).normalized;
        this.seed.motion *= planetaryRotationScalar;

        //for (int i = 0; i < this.seed.edges.Count; i++)
        //{
        //    this.seed.edges[i].motion = this.seed.motion;
        //}

        this.plateTiles.Add(seed);
        this.seed.tectonicPlate = this;
        if (this.plateType == CONTINENTAL)
        {
            int indexOfSeed = Globe.globeTiles.IndexOf(this.seed);
            int halfGlobeTileCount = Globe.globeTiles.Count / 2;
            int latitudeEstimate = Mathf.Abs(halfGlobeTileCount - indexOfSeed);


            if (latitudeEstimate < (halfGlobeTileCount / 5))
            {
                this.seed.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
            } else if (latitudeEstimate < (halfGlobeTileCount / 5) * 2)
            {
                this.seed.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
            } else if (latitudeEstimate < (halfGlobeTileCount / 5) * 3)
            {
                this.seed.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
            } else if (latitudeEstimate < (halfGlobeTileCount / 5) * 4)
            {
                this.seed.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
            } else if (latitudeEstimate < (halfGlobeTileCount / 5) * 5)
            {
                this.seed.terrain.GetComponent<MeshRenderer>().material = Globe.DesertMaterial;
            }
        } else
        {
            this.seed.terrain.GetComponent<MeshRenderer>().material = Globe.WaterMaterial;
        }
        //this.seed.terrain.GetComponent<MeshRenderer>().material.color = plateColor;
        this.smoothness = smoothness;


        // Determine and claim the seed's possible nieghbors
        for (int i = 0; i < this.plateTiles.Count; i++)
        {
            GlobeTile currentPlateTile = this.plateTiles[i];
            for (int j = 0; j < currentPlateTile.neighborTiles.Count; j++)
            {
                GlobeTile currentPlateTileNeighbor = currentPlateTile.neighborTiles[j];
                Vector3 currentPlateTileNeighborDelaunayPoint = currentPlateTileNeighbor.delaunayPoint;
                if (AllPlateTiles.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) == null)
                {
                    bool neighborIsClaimed = false;
                    for (int k = 0; k < AllPlates.Count; k++)
                    {
                        if (AllPlates[k].possibleNeighbors.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) != null)
                        {
                            neighborIsClaimed = true;
                        }
                    }
                    if (!neighborIsClaimed)
                    {
                        this.possibleNeighbors.Add(currentPlateTileNeighbor);
                    }
                }
            }
        }
    }

    public bool FindNextTiles()
    {
        if (possibleNeighbors.Count == 0)
        {
            // Determine perimeter tiles and perimeter neighbor tiles of this tectonic plate
            for (int i = 0; i < this.plateTiles.Count; i++)
            {
                GlobeTile currentPlateTile = this.plateTiles[i];
                for (int j = 0; j < currentPlateTile.neighborTiles.Count; j++)
                {
                    GlobeTile currentPlateTileNeighbor = currentPlateTile.neighborTiles[j];
                    if (currentPlateTileNeighbor.tectonicPlate == null || currentPlateTileNeighbor.tectonicPlate.id != this.id)
                    {
                        if (!this.perimeterTiles.Any(tile => tile.delaunayPoint == currentPlateTile.delaunayPoint))
                        {
                            currentPlateTile.tilesAwayFromPlatePerimeter = 0;
                            this.perimeterTiles.Add(currentPlateTile);
                        }
                        if (!this.perimeterTileNeighbors.Any(tile => tile.delaunayPoint == currentPlateTileNeighbor.delaunayPoint))
                        {
                            this.perimeterTileNeighbors.Add(currentPlateTileNeighbor);
                        }
                    }
                }
            }


            // Determine the perimeter edges of this tectonic plate
            List<GlobeTileEdge> allPerimeterTileEdges = new List<GlobeTileEdge>();
            List<GlobeTileEdge> allPerimeterTileNeighborEdges = new List<GlobeTileEdge>();
            for (int i = 0; i < this.perimeterTiles.Count; i++)
            {
                GlobeTile currentPerimeterTile = this.perimeterTiles[i];
                for (int j = 0; j < currentPerimeterTile.edges.Count; j++)
                {
                    allPerimeterTileEdges.Add(currentPerimeterTile.edges[j]);
                }
            }
            for (int i = 0; i < this.perimeterTileNeighbors.Count; i++)
            {
                GlobeTile currentPerimeterTileNeighbor = this.perimeterTileNeighbors[i];
                for (int j = 0; j < currentPerimeterTileNeighbor.edges.Count; j++)
                {
                    allPerimeterTileNeighborEdges.Add(currentPerimeterTileNeighbor.edges[j]);
                }
            }
            this.perimeterEdges = allPerimeterTileEdges.Intersect(allPerimeterTileNeighborEdges).ToList();

            // Determine the closest distance to a perimiter tile for each tile in this tectonic plate
            // TODO: FIX FUNCTION
            //GetNumberOfTilesAwayFromPerimeter(this.plateTiles[0], new List<GlobeTile>(), 0);

            return false;
        }

        // First attempt at tectonic plate genration
        //GlobeTile nextTile = possibleNeighbors[Random.Range(0, possibleNeighbors.Count)];
        //this.plateTiles.Add(nextTile);
        //AllPlateTiles.Add(nextTile);
        //nextTile.terrain.GetComponent<MeshRenderer>().material.color = this.plateColor;
        //possibleNeighbors.Clear();

        //for (int i = 0; i < this.plateTiles.Count; i++)
        //{
        //    GlobeTile currentPlateTile = this.plateTiles[i];
        //    for (int j = 0; j < currentPlateTile.neighborTiles.Count; j++)
        //    {
        //        GlobeTile currentPlateTileNeighbor = currentPlateTile.neighborTiles[j];
        //        Vector3 currentPlateTileNeighborDelaunayPoint = currentPlateTileNeighbor.delaunayPoint;
        //        if (AllPlateTiles.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) == null)
        //        {
        //            bool neighborIsClaimed = false;
        //            for (int k = 0; k < AllPlates.Count; k++)
        //            {
        //                if (AllPlates[k].possibleNeighbors.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) != null)
        //                {
        //                    neighborIsClaimed = true;
        //                }
        //            }
        //            if (!neighborIsClaimed)
        //            {
        //                int requiredAdjacencyCount = (Random.value * 100) <= this.smoothness ? 2 : 1;
        //                bool suitableNeighborFound = false;

        //                List<GlobeTile> adjacentPlateTiles = currentPlateTileNeighbor.neighborTiles.Intersect(this.plateTiles).ToList();
        //                for (int k = 0; k < adjacentPlateTiles.Count; k++)
        //                {
        //                    int possibleAdjacentNeighborCount = adjacentPlateTiles.Count;
        //                    if (possibleAdjacentNeighborCount >= requiredAdjacencyCount)
        //                    {
        //                        this.possibleNeighbors.Add(currentPlateTileNeighbor);
        //                        suitableNeighborFound = true;
        //                    }
        //                }
        //                if (!suitableNeighborFound)
        //                {
        //                    //this.possibleNeighbors.Add(currentPlateTileNeighbor);
        //                }
        //            }
        //        }
        //    }
        //}


        // Second attempt at tectonic plate generation
        //List<SuitableNeighborTile> suitableNeighborTiles = new List<SuitableNeighborTile>();
        //bool useHighestAdjacentcyCount = (Random.value * 100) <= this.smoothness ? true : false;
        //for (int i = 0; i < this.plateTiles.Count; i++)
        //{
        //    GlobeTile currentPlateTile = this.plateTiles[i];


        //    for (int j = 0; j < currentPlateTile.neighborTiles.Count; j++)
        //    {
        //        GlobeTile currentPlateTileNeighbor = currentPlateTile.neighborTiles[j];
        //        Vector3 currentPlateTileNeighborDelaunayPoint = currentPlateTileNeighbor.delaunayPoint;
        //        if (AllPlateTiles.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) == null)
        //        {
        //            bool neighborIsClaimed = false;
        //            for (int k = 0; k < AllPlates.Count; k++)
        //            {
        //                if (AllPlates[k].possibleNeighbors.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) != null)
        //                {
        //                    neighborIsClaimed = true;
        //                }
        //            }
        //            if (!neighborIsClaimed)
        //            {
        //                List<GlobeTile> adjacentPlateTiles = currentPlateTileNeighbor.neighborTiles.Intersect(this.plateTiles).ToList();
        //                suitableNeighborTiles.Add(new SuitableNeighborTile(adjacentPlateTiles.Count, currentPlateTileNeighbor));
        //            }
        //        }
        //    }
        //}
        //suitableNeighborTiles.Sort((a, b) => b.adjacencyCount.CompareTo(a.adjacencyCount));
        //if (suitableNeighborTiles.Count > 0)
        //{
        //    if (useHighestAdjacentcyCount)
        //    {
        //        this.possibleNeighbors.Add(suitableNeighborTiles[0].tile);
        //    }
        //    else
        //    {
        //        this.possibleNeighbors.Add(suitableNeighborTiles[Random.Range(0, suitableNeighborTiles.Count)].tile);
        //    }
        //    suitableNeighborTiles.Clear();
        //} else {
        //    return false;
        //}


        // Third Attempt at tectnoic plate generation 
        // Best determined algorithm for Tectonic Plate Generation
        List<GlobeTile> nextTiles = new List<GlobeTile>();
        int amountOfNextTilesToAdd = Mathf.CeilToInt(this.possibleNeighbors.Count * ((float)smoothness / 100f));
        for (int i = 0; i < amountOfNextTilesToAdd; i++)
        {
            int possibleNeighborIndex = Random.Range(0, possibleNeighbors.Count);
            GlobeTile nextTile = possibleNeighbors[Random.Range(0, possibleNeighbors.Count)];

            nextTile.motion = Vector3.Cross(nextTile.delaunayPoint, this.planetaryRotationAxis).normalized;
            nextTile.motion *= planetaryRotationScalar;

            Vector3 tileRotationalMotion = Vector3.Cross(this.seed.delaunayPoint, (this.seed.delaunayPoint - nextTile.delaunayPoint)) * plateRotationScalar;
            nextTile.motion += tileRotationalMotion;

            //for (int j = 0; j < nextTile.edges.Count; j++)
            //{
            //    nextTile.edges[j].motion = nextTile.motion;
            //}

            nextTile.tectonicPlate = this;
            possibleNeighbors.RemoveAt(possibleNeighborIndex);
            this.plateTiles.Add(nextTile);
            AllPlateTiles.Add(nextTile);
            nextTile.terrain.GetComponent<MeshRenderer>().material = this.seed.terrain.GetComponent<MeshRenderer>().material;
            //nextTile.terrain.GetComponent<MeshRenderer>().material.color = plateColor;

            List<GlobeTile> nextTilePlateNeighbors = nextTile.neighborTiles.FindAll(tile => tile.tectonicPlate == nextTile.tectonicPlate);
            nextTilePlateNeighbors.RemoveAll(tile => tile.tilesAwayFromPlateSeed < 0);
            nextTilePlateNeighbors.Sort((a, b) => a.tilesAwayFromPlateSeed.CompareTo(b.tilesAwayFromPlateSeed));
            int tilesAwayFromPlateSeed = nextTilePlateNeighbors[0].tilesAwayFromPlateSeed + 1;
            nextTile.tilesAwayFromPlateSeed = tilesAwayFromPlateSeed;
        }
        possibleNeighbors.Clear();


        for (int i = 0; i < this.plateTiles.Count; i++)
        {
            GlobeTile currentPlateTile = this.plateTiles[i];
            for (int j = 0; j < currentPlateTile.neighborTiles.Count; j++)
            {
                GlobeTile currentPlateTileNeighbor = currentPlateTile.neighborTiles[j];
                Vector3 currentPlateTileNeighborDelaunayPoint = currentPlateTileNeighbor.delaunayPoint;
                if (AllPlateTiles.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) == null)
                {
                    bool neighborIsClaimed = false;
                    for (int k = 0; k < AllPlates.Count; k++)
                    {
                        if (AllPlates[k].possibleNeighbors.Find(tile => tile.delaunayPoint == currentPlateTileNeighborDelaunayPoint) != null)
                        {
                            neighborIsClaimed = true;
                        }
                    }
                    if (!neighborIsClaimed)
                    {
                        this.possibleNeighbors.Add(currentPlateTileNeighbor);
                    }
                }
            }
        }

        return true;

    }


    ///<summary>
    ///Returns the adjacent perimeter tile in index 0 and corresponding neighbor tile in index 1 given a tectonic plate edge. Returns empty list if no tiles were found for given edge.
    ///</summary>
    public List<GlobeTile> FindAdjacentPerimeterAndNeighborTiles(GlobeTileEdge sharedEdge)
    {
        GlobeTile matchingPerimeterTile = this.perimeterTiles.Find(tile => tile.edges.Contains(sharedEdge));
        GlobeTile matchingPerimeterTileNeighbor = this.perimeterTileNeighbors.Find(tile => tile.edges.Contains(sharedEdge));

        List<GlobeTile> matchingTilePair = new List<GlobeTile>();
        if (matchingPerimeterTile != null)
        {
            matchingTilePair.Add(matchingPerimeterTile);
        }
        if (matchingPerimeterTileNeighbor != null)
        {
            matchingTilePair.Add(matchingPerimeterTileNeighbor);
        }
        return matchingTilePair;
    }

    ///<summary>
    ///Recursively calculates the distance in tiles of the closest perimeter tile.
    ///Returns the number of tiles the provided plateTile is away from the closest perimeter tile.
    ///This should only be called once the tectonic plates are all finished generating.
    ///<br></br>
    ///NOT WORKING YET - TODO: FIX
    ///</summary>
    private void GetNumberOfTilesAwayFromPerimeter(GlobeTile plateTile, List<GlobeTile> visitedTiles, int iterations)
    {
        if (this.perimeterTiles.Any(tile => tile.id == plateTile.id))
        {
            plateTile.tilesAwayFromPlatePerimeter = iterations;
            return;
        }
        if (visitedTiles.Count == this.plateTiles.Count)
        {
            return;
        }
        iterations++;
        visitedTiles.Add(plateTile);
        List<GlobeTile> nextPossiblePlateTileNeighbors = plateTile.neighborTiles.Except(visitedTiles).ToList();
        nextPossiblePlateTileNeighbors = nextPossiblePlateTileNeighbors.Except(this.perimeterTileNeighbors).ToList();

        for (int i = 0; i < nextPossiblePlateTileNeighbors.Count; i++)
        {
            GetNumberOfTilesAwayFromPerimeter(nextPossiblePlateTileNeighbors[i], visitedTiles, iterations);
        }


        return;
    }
}
