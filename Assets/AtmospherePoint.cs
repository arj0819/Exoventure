using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AtmospherePoint
{
    public static int AtmospherePointCount = 0;
    public int id = AtmospherePointCount;
    public static List<AtmospherePoint> AllPoints = new List<AtmospherePoint>();

    public Vector3 delaunayPoint;
    public Vector3 point;
    public List<AtmospherePoint> neighborPoints = new List<AtmospherePoint>();
    // SortedDictionary< % of moisture to transfer to the downwind neighbor, the downwind neighbor>
    public SortedDictionary<float, AtmospherePoint> downwindNeighbors = new SortedDictionary<float, AtmospherePoint>();
    public float totalDownwindFlow;
    public float scale;
    public float phi;
    public float theta;
    public GlobeTile globeTile;
    public float windDirectionScalar;
    public Vector3 windDirection;

    private Vector3 planetSpinAxis = Vector3.up.normalized;
    private float planetaryRotationScalar = Random.value + 0.1f;
    private Vector3 rotationAxis;
    private float plateRotationScalar = Random.value + 0.1f;
    public Color color;

    public float airMoisture;

    public AtmospherePoint(GlobeTile globeTile, int airMoisture)
    {
        AtmospherePointCount++;

        this.globeTile = globeTile;
        this.delaunayPoint = globeTile.delaunayPoint;
        this.scale = globeTile.scale + (globeTile.scale * 0.03f);
        this.point = this.delaunayPoint * this.scale;
        this.rotationAxis = this.point.normalized;
        this.phi = globeTile.phi;
        this.theta = globeTile.theta;
        // wind strength is the rate of change of moisture with respect to phi, which would be (0.5f * Mathf.Sin(6f * this.phi)) since moisture with respect to phi is (-0.5f * Mathf.Cos(6f * this.phi)) + 0.5f)
        this.windDirectionScalar = Mathf.Abs(0.5f * Mathf.Sin(6f * this.phi)) + 1f; // (adding 1f to the scalar guarantees moisture will move around)
        this.windDirection = GetWindDirection();
        this.windDirection *= this.windDirectionScalar;
        this.airMoisture = airMoisture;

        AllPoints.Add(this);
    }

    private Vector3 GetWindDirection()
    {
        Vector3 windDirection = Vector3.zero;
        float latitudinalTurbulence = Random.Range(0f, 1f);
        float longitudinalTurbulence = Random.Range(0f, 1f);

        if (this.phi <= Mathf.PI / 6)
        {
            longitudinalTurbulence = this.phi / (Mathf.PI / 6);
            latitudinalTurbulence = Mathf.Abs(1 - longitudinalTurbulence);

            windDirection = Vector3.Cross(this.planetSpinAxis, this.rotationAxis);
            windDirection *= longitudinalTurbulence;
            Vector3 verticalDirection = Vector3.Cross(windDirection, this.rotationAxis);
            windDirection += (verticalDirection * latitudinalTurbulence);

            this.color = new Color(255f, 0f, 0f);
        } else if (this.phi <= Mathf.PI / 3)
        {
            latitudinalTurbulence = (this.phi - (Mathf.PI / 6)) / (Mathf.PI / 6);
            longitudinalTurbulence = Mathf.Abs(1 - latitudinalTurbulence);

            windDirection = Vector3.Cross(this.rotationAxis, this.planetSpinAxis);
            windDirection *= longitudinalTurbulence;
            Vector3 verticalDirection = Vector3.Cross(this.rotationAxis, windDirection);
            windDirection -= (verticalDirection * latitudinalTurbulence);
            this.color = new Color(0f, 0f, 255f);
        }
        else if (this.phi <= Mathf.PI / 2)
        {
            longitudinalTurbulence = ((this.phi - (Mathf.PI / 3)) / (Mathf.PI / 6));
            latitudinalTurbulence = Mathf.Abs(1 - longitudinalTurbulence);

            windDirection = Vector3.Cross(this.planetSpinAxis, this.rotationAxis);
            windDirection *= longitudinalTurbulence;
            Vector3 verticalDirection = Vector3.Cross(windDirection, this.rotationAxis);
            windDirection += (verticalDirection * latitudinalTurbulence);
            this.color = new Color(255f, 0f, 0f);
        }
        else if (this.phi <= (Mathf.PI * 2) / 3)
        {
            latitudinalTurbulence = (this.phi - (Mathf.PI / 2)) / (Mathf.PI / 6);
            longitudinalTurbulence = Mathf.Abs(1 - latitudinalTurbulence);

            windDirection = Vector3.Cross(this.planetSpinAxis, this.rotationAxis);
            windDirection *= longitudinalTurbulence;
            Vector3 verticalDirection = Vector3.Cross(windDirection, this.rotationAxis);
            windDirection -= (verticalDirection * latitudinalTurbulence);
            this.color = new Color(255f, 0f, 0f);
        }
        else if (this.phi <= (Mathf.PI * 5) / 6)
        {
            longitudinalTurbulence = (this.phi - ((Mathf.PI * 2) / 3)) / (Mathf.PI / 6);
            latitudinalTurbulence = Mathf.Abs(1 - longitudinalTurbulence);

            windDirection = Vector3.Cross(this.rotationAxis, this.planetSpinAxis);
            windDirection *= longitudinalTurbulence;
            Vector3 verticalDirection = Vector3.Cross(this.rotationAxis, windDirection);
            windDirection += (verticalDirection * latitudinalTurbulence);
            this.color = new Color(0f, 0f, 255f);
        }
        else if (this.phi <= Mathf.PI)
        {
            latitudinalTurbulence = (this.phi - ((Mathf.PI * 5) / 6)) / (Mathf.PI / 6);
            longitudinalTurbulence = Mathf.Abs(1 - latitudinalTurbulence);

            windDirection = Vector3.Cross(this.planetSpinAxis, this.rotationAxis);
            windDirection *= longitudinalTurbulence;
            Vector3 verticalDirection = Vector3.Cross(windDirection, this.rotationAxis);
            windDirection -= (verticalDirection * latitudinalTurbulence);
            this.color = new Color(255f, 0f, 0f);
        }

        return windDirection.normalized;
    }
}
