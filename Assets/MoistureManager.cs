using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoistureManager
{
    public static float GlobalGroundMoistureReservior;
    public static float GlobalAirMoistureReservior;
    public static float RESERVIOR_FLUX = 0.05f;

    public static int MoistureManagerCount = 0;
    public int id = MoistureManagerCount;
    public GlobeTile globeTile;
    public AtmospherePoint atmospherePoint;
    public static List<MoistureManager> AllMoistureManagers = new List<MoistureManager>();

    public MoistureManager(GlobeTile globeTile, AtmospherePoint atmospherePoint)
    {
        this.globeTile = globeTile;
        this.globeTile.moistureManager = this;
        this.atmospherePoint = atmospherePoint;
        this.atmospherePoint.moistureManager = this;
        MoistureManagerCount++;
    }

    /// <summary>
    /// Calculates how much moisture is evaporated from the manager's GlobeTile to the manager's AtmospherePoint, then moves it.
    /// </summary>
    public void Evaporate()
    {

    }

    /// <summary>
    /// Calculates how much moisture is precipitated from the manager's AtmospherePoint to the manager's GlobeTile, then moves it.
    /// </summary>
    public void Precipitate()
    {

    }

    /// <summary>
    /// Calculates how much moisture is percolated from the manager's GlobeTile to its neighboring GlobeTiles, then moves it.
    /// Percolate only performs on LAND tiles, but will move moisture into WATER tiles if starting from a LAND tile.
    /// </summary>
    public void Percolate()
    {

    }
}
