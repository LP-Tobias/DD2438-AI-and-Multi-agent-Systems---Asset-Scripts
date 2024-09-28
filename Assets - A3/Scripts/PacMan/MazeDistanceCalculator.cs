using Scripts.Map;
using System.Collections.Generic;
using UnityEngine;
using System;
public class MazeDistanceCalculator
{
    private Dictionary<Vector3, int>[,] distanceMap;
    private float cellSize;
    private ObstacleMap _map;
    public int gridMinX, gridMaxX, gridMinY, gridMaxY;

    public void CalculateDistances(ObstacleMap map, float size, int minX, int maxX, int minY, int maxY)
    {
        _map = map;
        cellSize = size;
        gridMinX = minX;
        gridMaxX = maxX;
        gridMinY = minY;
        gridMaxY = maxY;

        int width = Mathf.RoundToInt((gridMaxX - gridMinX) / cellSize) + 1;
        int height = Mathf.RoundToInt((gridMaxY - gridMinY) / cellSize) + 1;
        distanceMap = new Dictionary<Vector3, int>[width, height];

        // gridMinX: -23, gridMaxX: 23, gridMinY: -13, gridMaxY: 14
        for (float x = gridMinX; x < gridMaxX; x += cellSize)
        {
            for (float y = gridMinY; y < gridMaxY; y += cellSize)
            {
                Vector3 cellCenter = new Vector3(x + cellSize / 2f, 0f, y + cellSize / 2f);
                ObstacleMap.Traversability tv = _map.IsGlobalPointTraversable(cellCenter);
                // Debug.Log("cell center: " + cellCenter + " tv: " + tv);
                if (tv == ObstacleMap.Traversability.Free)
                {
                    CalculateDistancesFromCell(cellCenter);
                }
            }
        }
    }

    public Vector3 GetClosestCenterCell(Vector3 position, bool red = true)
    {
        float distToCenter = float.MaxValue;
        Vector3 bestPos = Vector3.zero;
        for (int i = gridMinY; i < gridMaxY; i += 1)
        {
            Vector3 pos = new(red ? 2.5f : -2.5f, 0, i + 0.5f);
            float dist = GetDistance(position, pos);
            //Debug.Log("disst from : " + position + " to : " + pos + " = " + dist);
            if (dist != -1 && dist < distToCenter)
            {
                distToCenter = dist;
                bestPos = pos;
            }
        }
        return bestPos;
    }

    public List<Vector3> GetClosestPathToCenter(Vector3 position, bool red= true)
    {
        float distToCenter = float.MaxValue;
        Vector3 bestPos = Vector3.zero; 
        for (int i = gridMinY; i < gridMaxY; i += 1)
        {
            Vector3 pos = new(red?0.5f:-0.5f, 0, i+0.5f);
            float dist = GetDistance(position, pos);
            //Debug.Log("disst from : " + position + " to : " + pos + " = " + dist);
            if (dist!=-1 && dist < distToCenter)
            {
                distToCenter = dist; 
                bestPos = pos;
            }
        }
        //Debug.Log("path to center from : " + position + " best dist is : " + distToCenter + " the best pos is : " + bestPos);
        return GetPath(position, bestPos);
    }

    private void CalculateDistancesFromCell(Vector3 startCell)
    {
        Queue<Vector3> queue = new Queue<Vector3>();
        HashSet<Vector3> visited = new HashSet<Vector3>();
        Dictionary<Vector3, int> distances = new Dictionary<Vector3, int>();

        queue.Enqueue(startCell);
        visited.Add(startCell);
        distances[startCell] = 0;

        while (queue.Count > 0)
        {
            Vector3 currentCell = queue.Dequeue();
            int currentDistance = distances[currentCell];

            Vector3[] neighbors = GetNeighbors(currentCell);
            foreach (Vector3 neighbor in neighbors)
            {
                if (IsPositionInMap(neighbor) && _map.IsGlobalPointTraversable(neighbor) == ObstacleMap.Traversability.Free && !visited.Contains(neighbor))
                {
                    queue.Enqueue(neighbor);
                    visited.Add(neighbor);
                    distances[neighbor] = currentDistance + 1;
                }
            }
        }

        int cellX = Mathf.FloorToInt((startCell.x - gridMinX) / cellSize);
        int cellY = Mathf.FloorToInt((startCell.z - gridMinY) / cellSize);

        // Debug.Log("cell added : " + cellX + ", " + cellY + " from start: " + startCell);
        distanceMap[cellX, cellY] = distances;
    }

    private Vector3[] GetNeighbors(Vector3 cell)
    {
        return new Vector3[]
        {
            new Vector3(cell.x - cellSize, 0f, cell.z),
            new Vector3(cell.x + cellSize, 0f, cell.z),
            new Vector3(cell.x, 0f, cell.z - cellSize),
            new Vector3(cell.x, 0f, cell.z + cellSize)
        };
    }

    private bool IsPositionInMap(Vector3 position)
    {
        return position.x >= gridMinX && position.x <= gridMaxX && position.z >= gridMinY && position.z <= gridMaxY;
    }

    public int GetDistance(Vector3 startCell, Vector3 endCell)
    {
        endCell.y = 0; 
        startCell.x = RoundToNearestHalf(startCell.x);
        startCell.z = RoundToNearestHalf(startCell.z);
        endCell.x = RoundToNearestHalf(endCell.x);
        endCell.z = RoundToNearestHalf(endCell.z);

        int startX = Mathf.FloorToInt((startCell.x - gridMinX) / cellSize);
        int startY = Mathf.FloorToInt((startCell.z - gridMinY) / cellSize);
        int endX = Mathf.FloorToInt((endCell.x - gridMinX) / cellSize);
        int endY = Mathf.FloorToInt((endCell.z - gridMinY) / cellSize);

        if (startX < 0 || startX >= distanceMap.GetLength(0) || startY < 0 || startY >= distanceMap.GetLength(1) ||
            endX < 0 || endX >= distanceMap.GetLength(0) || endY < 0 || endY >= distanceMap.GetLength(1))
        {
            //Debug.Log("1. disttt return -1, startCell: " + startCell + " ;;end cell: " + endCell + " , startX: " + startX + " endX: " + endX + " startY: " + startY + " endY: " + endY);
            return -1; // Invalid cell positions
        }

        if (distanceMap[startX, startY] == null)
        {
            //Debug.Log("2. disttt return -1, startCell: "+startCell+" ;;end cell: "+endCell+" , startX: " + startX + " endX: " + endX + " startY: " + startY + " endY: " + endY);
            return -1; // No path exists between the cells
        }

        if (!distanceMap[startX, startY].ContainsKey(endCell))
        {
            //Debug.Log("3. disttt return -1, startCell: " + startCell + " ;;end cell: " + endCell + " , startX: " + startX + " endX: " + endX + " startY: " + startY + " endY: " + endY);
            return -1; // No path exists between the cells
        }

        return distanceMap[startX, startY][endCell];
    }

    public List<Vector3> GetPath(Vector3 startCell, Vector3 endCell)
    {
        endCell.y = 0;
        startCell.x = RoundToNearestHalf(startCell.x);
        startCell.z = RoundToNearestHalf(startCell.z);
        endCell.x = RoundToNearestHalf(endCell.x);
        endCell.z = RoundToNearestHalf(endCell.z);

        List<Vector3> path = new List<Vector3>();
        int startX = Mathf.FloorToInt((startCell.x - gridMinX) / cellSize);
        int startY = Mathf.FloorToInt((startCell.z - gridMinY) / cellSize);
        int endX = Mathf.FloorToInt((endCell.x - gridMinX) / cellSize);
        int endY = Mathf.FloorToInt((endCell.z - gridMinY) / cellSize);

        if (startX < 0 || startX >= distanceMap.GetLength(0) || startY < 0 || startY >= distanceMap.GetLength(1) ||
            endX < 0 || endX >= distanceMap.GetLength(0) || endY < 0 || endY >= distanceMap.GetLength(1))
        {
            return path; // Invalid cell positions
        }

        if (distanceMap[startX, startY] == null || !distanceMap[startX, startY].ContainsKey(endCell))
        {
            return path; // No path exists between the cells
        }

        Vector3 currentCell = endCell;
        int i = 100; 
        while (currentCell != startCell)
        {
            path.Add(currentCell);
            Vector3[] neighbors = GetNeighbors(currentCell);
            int minDistance = int.MaxValue;
            Vector3 nextCell = currentCell;

            foreach (Vector3 neighbor in neighbors)
            {
                if (distanceMap[startX, startY].TryGetValue(neighbor, out int neighborDistance) && neighborDistance < minDistance)
                {
                    minDistance = neighborDistance;
                    nextCell = neighbor;
                }
            }

            if (currentCell == nextCell)
            {
                break; // Stuck in a loop, no path found
            }

            currentCell = nextCell;
            i -= 1;
            if (i == 0) break; 
        }

        path.Add(startCell);
        path.Reverse(); // Reverse the list to get the path from start to end
        return path;
    }

    public static float RoundToNearestHalf(float number)
    {
        float lower = (float)Math.Floor(number);
        float upper = lower + 1.0f;
        float lowerHalf = lower + 0.5f;
        float upperHalf = upper - 0.5f;

        if ((int)number == number)
        {
            return number - 0.5f;
        }
        if (number < lowerHalf)
        {
            return number <= lowerHalf - 0.5f ? lower : lowerHalf;
        }
        else
        {
            return number >= upperHalf + 0.5f ? upper : upperHalf;
        }
    }
}