using Scripts.Map;
using System;
using System.Collections.Generic;
using UnityEngine;

public static class VehicleUtils
{
    public static float EuclidianDist(Vector3 ptA, Vector3 ptB)
    {

        return Mathf.Sqrt(Mathf.Pow(ptA.x - ptB.x, 2) + Mathf.Pow(ptA.z - ptA.z, 2));
    }
    public static void PrintPath(List<Vector3> path, MapManager mapManager)
    {
        Debug.Log("-----printing path-----, size of path = " + path.Count);
        //for (int i = 0; i < path.Count - 1; i++)
        //{
        //    // Debug.Log("index i : " + i + " transformed: " + mapManager.grid.LocalToWorld(path[i])); // scales back to real coordinate
        //    Debug.Log("distance from " + mapManager.grid.LocalToWorld(path[i]) + " to : " + mapManager.grid.LocalToWorld(path[i+1])+ " : "+
        //        EuclidianDist(mapManager.grid.LocalToWorld(path[i]), mapManager.grid.LocalToWorld(path[i+i])));
        //}
        foreach (var point in path)
        {
            Debug.Log("pt:" + point);
        }
    }

    public static float CalculatePathLength(List<Vector3> path)
    {
        float totalLength = 0;
        for (int i = 1; i < path.Count; i++)
        {
            totalLength += Vector3.Distance(path[i], path[i - 1]);
        }
        return totalLength;
    }

    public static void PlotPathWorld(List<Vector3> path, Color lineColor, Color pointColor, bool drawPoints = true)
    {
        Vector3 old_wp = path[0];

        // For persistent points
        if (drawPoints)
        {
            foreach (var point in path)
            {
                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.position = point;
                sphere.transform.localScale = new Vector3(.5f, .5f, .5f); var collider = sphere.GetComponent<Collider>();
                if (collider != null)
                {
                    collider.enabled = false;
                }

                var renderer = sphere.GetComponent<Renderer>();
                if (renderer != null)
                {
                    renderer.material.color = pointColor;
                }
                // Optionally, make these spheres child of a specific GameObject to keep the hierarchy organized
            }
        }

        // Drawing lines
        for (int j = 0; j < path.Count; j++)
        {
            var wp = path[j];
            for (int i = 0; i < 20; i++)
            {
                Debug.DrawLine(old_wp, wp, lineColor, 1000f);
            }
            old_wp = wp;
        }
    }

    public static void PlotSphere(Vector3 point, MapManager mp, Color c)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = mp.grid.LocalToWorld(point);
        sphere.transform.localScale = new Vector3(1.5f, 1.5f, 1.5f); var collider = sphere.GetComponent<Collider>();
        if (collider != null)
        {
            collider.enabled = false;
        }

        var renderer = sphere.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = c;
        }
    }

    public static List<Vector3> RefinePath(List<Vector3> path, double maxGap, MapManager mapManager)
    {
        if (path == null || path.Count < 2)
            return path; // No refinement needed for empty or single-point paths

        List<Vector3> refinedPath = new List<Vector3>(path);
        int i = 0;
        while (i < refinedPath.Count - 1)
        {
            Vector3 currentPoint = refinedPath[i];
            Vector3 nextPoint = refinedPath[i + 1];

            //Debug.Log("current : " + refinedPath[i] + " next : " + refinedPath[i + 1]);
            //Debug.Log("world ; current : " + currentPoint + " next: " + nextPoint);

            double distance = Vector3.Distance(currentPoint, nextPoint);
            //Debug.Log("distance : " + distance);

            if (distance > maxGap)
            {
                // Calculate midpoint
                Vector3 midpoint = new Vector3(
                    refinedPath[i].x - (refinedPath[i].x - refinedPath[i + 1].x) / 2,
                    refinedPath[i].y - (refinedPath[i].y - refinedPath[i + 1].y) / 2,
                    refinedPath[i].z - (refinedPath[i].z - refinedPath[i + 1].z) / 2);

                //Debug.Log("mid point : " + midpoint);
                //Debug.Log("world mid point : " + mapManager.grid.LocalToWorld(midpoint));
                // Insert midpoint into the list, do not increment i to check the new segment next
                refinedPath.Insert(i + 1, midpoint);
            }
            else
            {
                // Move to the next pair if no insertion was made
                i++;
            }
        }

        return refinedPath;
    }

    //public static List<Vector3> GetEvenlySpacedPoints(List<Vector3> points, int nbPoints)
    //{
    //    if (points.Count < 2 || nbPoints <= 0)
    //    {
    //        Debug.LogError("Invalid input: List must have at least 2 points and nbPoints must be positive.");
    //        return new List<Vector3>();
    //    }

    //    List<Vector3> newPoints = new List<Vector3>();
    //    newPoints.Add(points[0]); // Add first point

    //    if (nbPoints > 0)
    //    {
    //        Vector3 direction = points[points.Count - 1] - points[0];
    //        float segmentLength = direction.magnitude;
    //        float stepSize = segmentLength / (nbPoints + 1); // Include both ends

    //        for (int i = 1; i <= nbPoints; i++)
    //        {
    //            Vector3 newPoint = points[0] + direction * (i * stepSize / segmentLength);
    //            newPoints.Add(newPoint);
    //        }
    //    }

    //    newPoints.Add(points[points.Count - 1]); // Add last point

    //    return newPoints;
    //}

    public static List<Vector3> GetEvenlySpacedPoints(List<Vector3> points, int nbPoints)
    {
        if (points.Count < 2 || nbPoints <= points.Count)
        {
            Debug.Log("Invalid input: List must have at least 2 points and nbPoints must be greater than the number of original points.");
            return points;
        }

        List<Vector3> newPoints = new List<Vector3>();

        float totalLength = GetTotalLength(points);
        float segmentLength = totalLength / (nbPoints - 1);
        float accumulatedLength = 0f;
        int currentPointIndex = 0;
        newPoints.Add(points[currentPointIndex]);

        for (int i = 1; i < nbPoints - 1; i++)
        {
            float targetLength = i * segmentLength;
            while (accumulatedLength < targetLength)
            {
                currentPointIndex++;
                float distanceToNextPoint = Vector3.Distance(points[currentPointIndex - 1], points[currentPointIndex]);
                accumulatedLength += distanceToNextPoint;
            }

            float overshoot = accumulatedLength - targetLength;
            Vector3 startPoint = points[currentPointIndex - 1];
            Vector3 endPoint = points[currentPointIndex];
            Vector3 direction = (endPoint - startPoint).normalized;
            Vector3 newPoint = endPoint - direction * overshoot;
            newPoints.Add(newPoint);
        }

        newPoints.Add(points[points.Count - 1]); // Add last point

        return newPoints;
    }

    private static float GetTotalLength(List<Vector3> points)
    {
        float totalLength = 0f;
        for (int i = 0; i < points.Count - 1; i++)
        {
            totalLength += Vector3.Distance(points[i], points[i + 1]);
        }
        return totalLength;
    }

    public static List<Vector3> EvenlyPlacePointsOnPath(List<Vector3> path, int nbPoints)
    {
        if (nbPoints <= path.Count)
        {
            return path; // No need to add points if desired number is less than or equal to existing points
        }

        List<Vector3> newPath = new List<Vector3>(path);
        float totalDistance = 0;

        // Calculate the total distance between points
        for (int i = 1; i < path.Count; i++)
        {
            totalDistance += Vector3.Distance(path[i], path[i - 1]);
        }

        // Calculate the distance between each new point
        float segmentLength = totalDistance / (nbPoints - path.Count);

        // Add new points based on accumulated distance
        float accumulatedDistance = 0;
        Vector3 previousPoint = path[0];
        for (int i = 1; i < nbPoints; i++)
        {
            if (accumulatedDistance >= totalDistance)
            {
                break;
            }

            // Find the point on the path segment where the accumulated distance matches
            float t = Mathf.Clamp01(accumulatedDistance / segmentLength);
            Vector3 newPoint = Vector3.Lerp(previousPoint, path[Mathf.Min(path.Count - 1, Mathf.FloorToInt(accumulatedDistance / segmentLength))], t);

            newPath.Insert(i, newPoint);
            previousPoint = newPoint;
            accumulatedDistance += segmentLength;
        }

        return newPath;
    }

    public static List<Vector3> AddPointsOnTurns(List<Vector3> path, double thresholdAngle, double distance)
    {
        if (path == null || path.Count < 3)
            return path; // No action needed for paths with less than three points

        List<Vector3> enhancedPath = new List<Vector3>();
        enhancedPath.Add(path[0]); // Always add the first point

        for (int i = 1; i < path.Count - 1; i++)
        {
            Vector3 prevPoint = path[i - 1];
            Vector3 currPoint = path[i];
            Vector3 nextPoint = path[i + 1];

            // Calculate vectors from the current point to the previous and next points
            Vector3 vectorToPrev = prevPoint - currPoint;
            Vector3 vectorToNext = nextPoint - currPoint;

            // Calculate distances from the current point to the previous and next points
            double distanceToPrev = Vector3.Distance(currPoint, prevPoint);
            double distanceToNext = Vector3.Distance(currPoint, nextPoint);

            // Calculate the angle between the vectors
            double angle = Math.Acos(Vector3.Dot(Vector3.Normalize(vectorToPrev), Vector3.Normalize(vectorToNext))) * (180.0 / Math.PI);

            if (angle < thresholdAngle)
            {
                // Check if there's enough room to add a new point before and after the current point
                if (distanceToPrev > distance)
                {
                    Vector3 newPointBefore = currPoint + Vector3.Normalize(vectorToPrev) * (float)distance;
                    enhancedPath.Add(newPointBefore); // Add a new point at 'distance' before the current point
                }
                else
                {
                    // If not enough room, add the current point directly
                    enhancedPath.Add(currPoint);
                }

                if (distanceToNext > distance)
                {
                    Vector3 newPointAfter = currPoint + Vector3.Normalize(vectorToNext) * (float)distance;
                    enhancedPath.Add(newPointAfter); // Add a new point at 'distance' after the current point
                }
                // Note: The current point is not added here if a new point after it is added to avoid duplicate additions.
            }
            else
            {
                // If the angle is not sharp enough, just add the current point
                enhancedPath.Add(currPoint);
            }
        }

        // Always add the last point
        enhancedPath.Add(path[path.Count - 1]);

        return enhancedPath;
    }

    public static List<Vector3> AddPointsOnTurns2(List<Vector3> path, double thresholdAngle, double distance)
    {
        if (path == null || path.Count < 3)
            return path; // No action needed for paths with less than three points

        List<Vector3> enhancedPath = new List<Vector3>();

        // Initially, add the first point to the enhanced path
        enhancedPath.Add(path[0]);

        int i = 1; // Start from the second point in the path
        while (i < path.Count - 1)
        {
            Vector3 prevPoint = path[i - 1];
            Vector3 currPoint = path[i];
            Vector3 nextPoint = path[i + 1];

            // Calculate vectors for direction
            Vector3 vectorPrevCurr = currPoint - prevPoint;
            Vector3 vectorCurrNext = nextPoint - currPoint;

            // Normalize vectors for angle calculation
            Vector3 vectorPrevCurrNormalized = Vector3.Normalize(vectorPrevCurr);
            Vector3 vectorCurrNextNormalized = Vector3.Normalize(vectorCurrNext);

            // Calculate the angle between the vectors
            double angle = Math.Acos(Vector3.Dot(vectorPrevCurrNormalized, vectorCurrNextNormalized)) * (180.0 / Math.PI);

            if (angle < thresholdAngle)
            {
                // Add a new point before the current point if distance allows
                if (Vector3.Distance(prevPoint, currPoint) > distance)
                {
                    Vector3 newPointBefore = currPoint - vectorPrevCurrNormalized * (float)distance;
                    enhancedPath.Add(newPointBefore);
                }

                // Add the current point
                enhancedPath.Add(currPoint);

                // Add a new point after the current point if distance allows
                if (Vector3.Distance(currPoint, nextPoint) > distance)
                {
                    Vector3 newPointAfter = currPoint + vectorCurrNextNormalized * (float)distance;
                    enhancedPath.Add(newPointAfter);
                }
            }
            else
            {
                // If the angle is not sharp enough, just add the current point
                // This ensures all original points are kept
                enhancedPath.Add(currPoint);
            }

            i++; // Move to the next point in the path
        }

        // Ensure the last point is added if it hasn't been added already
        if (enhancedPath[enhancedPath.Count - 1] != path[path.Count - 1])
        {
            enhancedPath.Add(path[path.Count - 1]);
        }

        return enhancedPath;
    }

    public static float EuclidianWorldDistance(Vector3 vector1, Vector3 vector2)
    {
        return Mathf.Sqrt(
            Mathf.Pow(vector1.x - vector2.x, 2) +
            Mathf.Pow(vector1.z - vector2.z, 2));
    }
}