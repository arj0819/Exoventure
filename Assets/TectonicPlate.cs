using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class TectonicPlate
{
    public static int TotalTectonicPlateCount = 0;
    public const int OCEANIC = 0;
    public const int CONTINENTAL = 1;
    public const float MIN_DENSITY = 2.7f;
    public const float MAX_DENSITY = 3.0f;
    public const float MANTLE_DENSITY = 5.5f;
    public const float CRUST_RADIUS_THICKNESS_RATIO = 0.02f;

    public int id = TotalTectonicPlateCount;
    public int plateType;
    private static List<TectonicPlate> AllPlates = new List<TectonicPlate>();
    private static List<GlobeTile> AllPlateTiles = new List<GlobeTile>();
    public int maxDistanceFromPerimeter = 0;
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

    public float defaultElevation; // The starting elevation of this plate. Calculated in the constructor
    public float density = Random.Range(MIN_DENSITY, MAX_DENSITY); // Real avg density of Earth continental lithosphere is 2.7g/cm^3, oceanic lithosphere is 3g/cm^3
    public static float CrustThickness;
    public static float TectonicActvity;

    public TectonicPlate(GlobeTile seed, int smoothness, float tectonicActivity, float globeRadius)
    {
        if (smoothness > 100)
        {
            smoothness = 100;
        }
        else if (smoothness < 0)
        {
            smoothness = 0;
        }
        TectonicPlate.CrustThickness = CRUST_RADIUS_THICKNESS_RATIO * globeRadius;
        TectonicPlate.TectonicActvity = tectonicActivity;
        this.defaultElevation = globeRadius - ((this.density / MANTLE_DENSITY) * TectonicPlate.CrustThickness);

        TotalTectonicPlateCount++;
        AllPlates.Add(this);
        AllPlateTiles.Add(seed);
        this.seed = seed;
        this.seed.tilesAwayFromPlateSeed = 0;
        this.seed.SetElevation(this.defaultElevation);
        this.plateRotationAxis = this.seed.delaunayPoint.normalized;
        this.seed.motion = Vector3.Cross(this.seed.delaunayPoint, this.planetaryRotationAxis).normalized;
        this.seed.motion *= planetaryRotationScalar;
        this.seed.motion *= TectonicPlate.TectonicActvity;


        this.plateTiles.Add(seed);
        this.seed.tectonicPlate = this;
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
            for (int i = 0; i < this.plateTiles.Count; i++)
            {
                Queue<GlobeTile> queuedTiles = new Queue<GlobeTile>();
                List<GlobeTile> visitedTiles = new List<GlobeTile>();
                List<GlobeTile> closestPerimeterTiles = new List<GlobeTile>();
                int depth = 0;

                queuedTiles.Enqueue(this.plateTiles[i]);

                while (queuedTiles.Count > 0)
                {
                    int tilesInCurrentLevel = queuedTiles.Count;
                    bool foundPerimeter = false;
                    while (tilesInCurrentLevel-- > 0)
                    {
                        GlobeTile nextTileInLevel = queuedTiles.Dequeue();
                        GlobeTile possibleClosestTectonicPerimiterTile = this.perimeterTiles.Find(tile => tile.id == nextTileInLevel.id);
                        if (possibleClosestTectonicPerimiterTile != null)
                        {
                            plateTiles[i].closestPerimiterTiles.Add(possibleClosestTectonicPerimiterTile);
                            foundPerimeter = true;
                        }
                        visitedTiles.Add(nextTileInLevel);
                        List<GlobeTile> nextPossibleNeighbors = nextTileInLevel.neighborTiles.Except(visitedTiles).ToList();
                        nextPossibleNeighbors = nextPossibleNeighbors.Except(this.perimeterTileNeighbors).ToList();
                        GlobeTile.ShuffleTiles(nextPossibleNeighbors);
                        for (int j = 0; j < nextPossibleNeighbors.Count; j++)
                        {
                            queuedTiles.Enqueue(nextPossibleNeighbors[j]);
                        }
                    }
                    if (foundPerimeter)
                    {
                        break;
                    }
                    depth++;
                }
                this.plateTiles[i].tilesAwayFromPlatePerimeter = depth;
                if (depth > this.maxDistanceFromPerimeter)
                {
                    this.maxDistanceFromPerimeter = depth;
                }
            }

            return false;
        }

        // Third Attempt at tectnoic plate generation 
        // Best determined algorithm for Tectonic Plate Generation
        List<GlobeTile> nextTiles = new List<GlobeTile>();
        int amountOfNextTilesToAdd = Mathf.CeilToInt(this.possibleNeighbors.Count * ((float)smoothness / 100f));
        for (int i = 0; i < amountOfNextTilesToAdd; i++)
        {
            int possibleNeighborIndex = Random.Range(0, possibleNeighbors.Count);
            GlobeTile nextTile = possibleNeighbors[possibleNeighborIndex];

            // Check if this tile was already claimed by another plate. If so, just skip to the next tile
            if (nextTile.tectonicPlate != null)
            {
                continue;
            }

            nextTile.SetElevation(this.defaultElevation);

            nextTile.motion = Vector3.Cross(nextTile.delaunayPoint, this.planetaryRotationAxis).normalized;
            nextTile.motion *= planetaryRotationScalar;

            Vector3 tileRotationalMotion = Vector3.Cross(this.seed.delaunayPoint, (this.seed.delaunayPoint - nextTile.delaunayPoint)) * plateRotationScalar;
            nextTile.motion += tileRotationalMotion;
            nextTile.motion *= TectonicPlate.TectonicActvity;

            nextTile.tectonicPlate = this;
            possibleNeighbors.RemoveAt(possibleNeighborIndex);
            this.plateTiles.Add(nextTile);
            AllPlateTiles.Add(nextTile);
            nextTile.terrain.GetComponent<MeshRenderer>().material = this.seed.terrain.GetComponent<MeshRenderer>().material;

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
                if (currentPlateTileNeighbor.tectonicPlate != null)
                {
                    continue;
                }

                if (AllPlateTiles.Find(tile => tile.id == currentPlateTileNeighbor.id) == null)
                {
                    this.possibleNeighbors.Add(currentPlateTileNeighbor);
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
}
