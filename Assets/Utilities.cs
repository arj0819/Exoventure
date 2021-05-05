using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utilities
{
    public static float GetPrecipitationRate(float phi)
    {
        float a1 = 0.5226f;
        float b1 = 1.57f;
        float c1 = 0.1104f;
        float a2 = -0.1455f;
        float b2 = 1.502f;
        float c2 = 0.2824f;
        float a3 = -961.8f;
        float b3 = 1.568f;
        float c3 = 0.3752f;
        float a4 = 961.2f;
        float b4 = 1.568f;
        float c4 = 0.3749f;
        float a5 = -0.02236f;
        float b5 = 1.175f;
        float c5 = 1.305f;
        float a6 = 1.219f;
        float b6 = 1.564f;
        float c6 = 0.756f;

        return a1 * Mathf.Exp(-1f * Mathf.Pow((phi - b1) / c1, 2f)) +
               a2 * Mathf.Exp(-1f * Mathf.Pow((phi - b2) / c2, 2f)) +
               a3 * Mathf.Exp(-1f * Mathf.Pow((phi - b3) / c3, 2f)) +
               a4 * Mathf.Exp(-1f * Mathf.Pow((phi - b4) / c4, 2f)) +
               a5 * Mathf.Exp(-1f * Mathf.Pow((phi - b5) / c5, 2f)) +
               a6 * Mathf.Exp(-1f * Mathf.Pow((phi - b6) / c6, 2f));

    }
    public static float GetEvaporationRate(float phi)
    {
        float a1 = 0.1338f;
        float b1 = 1.827f;
        float c1 = 0.2041f;
        float a2 = 0.058f;
        float b2 = 2.528f;
        float c2 = 0.2898f;
        float a3 = -0.06719f;
        float b3 = 1.568f;
        float c3 = 0.06087f;
        float a4 = 0.4734f;
        float b4 = 1.354f;
        float c4 = 0.5838f;
        float a5 = 0.2326f;
        float b5 = 2.105f;
        float c5 = 0.2987f;

        return a1 * Mathf.Exp(-1f * Mathf.Pow((phi - b1) / c1, 2)) +
               a2 * Mathf.Exp(-1f * Mathf.Pow((phi - b2) / c2, 2)) +
               a3 * Mathf.Exp(-1f * Mathf.Pow((phi - b3) / c3, 2)) +
               a4 * Mathf.Exp(-1f * Mathf.Pow((phi - b4) / c4, 2)) +
               a5 * Mathf.Exp(-1f * Mathf.Pow((phi - b5) / c5, 2));
    }

    public static float NextRandomNormal(float mean, float stdDev) {
        float u1 = 1.0f - Random.value;
        float u2 = 1.0f - Random.value;
        float randomStandardNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return mean + (stdDev * randomStandardNormal);
        
    }
}
