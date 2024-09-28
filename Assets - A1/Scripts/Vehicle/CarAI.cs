using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using UnityEngine.Tilemaps;

using System.Linq;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;
        private List<Vector3> my_path = new List<Vector3>(); 
        private List<Vector3> my_path2 = new List<Vector3>(); 
        private Dictionary<Vector2, Node> graph;
        Rigidbody my_rigidbody;

        public class Node {
            public Vector2 GridPosition { get; set; }
            public List<Node> Neighbors { get; set; }

            public Node(Vector2 gridPos) {
                GridPosition = gridPos;
                Neighbors = new List<Node>();
            }

            public static float Distance(Node a, Node b) {
                return Vector2.Distance(a.GridPosition, b.GridPosition);
            }

            public static Vector2 RoundVector2(Vector2 vector) {
                return new Vector2((float)Math.Round(vector.x, 3), (float)Math.Round(vector.y, 3));
            }
        }

        public class State {
            public Node CurrentNode;
            public float Cost;
            public State Parent;
            public float TotalEstimate;

            public State(Node node, float cost, State parent) {
                CurrentNode = node;
                Cost = cost;
                Parent = parent;
            }
        }

        public class StatePriorityQueue {
            private List<State> elements = new List<State>();

            public int Count => elements.Count;

            public void Add(State state) {
                elements.Add(state);
                elements.Sort((a, b) => a.TotalEstimate.CompareTo(b.TotalEstimate));
            }

            public State Pop() {
                var item = elements[0];
                elements.RemoveAt(0);
                return item;
            }
        }


        public void InitializeGraph(MapManager mapManager) {
            graph = new Dictionary<Vector2, Node>();
            ObstacleMap obstacleMap = mapManager.GetObstacleMap();
            float girdSizeX = mapManager.grid.cellSize.x;
            float girdSizeY = mapManager.grid.cellSize.y;
            float girdSizeZ = mapManager.grid.cellSize.z;
            
            foreach (var posThreeDim in obstacleMap.mapBounds.allPositionsWithin) { 
                ObstacleMap.Traversability status = obstacleMap.IsLocalPointTraversable(posThreeDim);

                if (status != ObstacleMap.Traversability.Blocked ) {
                    Vector3 cellCenter = new Vector3(posThreeDim.x * girdSizeX + girdSizeX/2, posThreeDim.y * girdSizeY, posThreeDim.z * girdSizeZ + girdSizeZ/2);
                    Vector3 interpolatedPos = cellCenter;
                    List<GameObject> objectsInCell = obstacleMap.gameGameObjectsPerCell[new Vector2Int((int)posThreeDim.x, (int)posThreeDim.z)];

                    bool isCenterFree = true;
                    if(status == ObstacleMap.Traversability.Partial){
                        foreach (GameObject obj in objectsInCell) {
                            // Convert the world bounds of the object to local bounds
                            Bounds worldBounds = obj.GetComponent<Renderer>().bounds;
                            Bounds localBounds = new Bounds(
                                obstacleMap.grid.WorldToLocal(worldBounds.center), 
                                obstacleMap.grid.WorldToLocal(worldBounds.size)
                            );
                            // Draw the white box and stay for 15sec
                            // DrawBounds(localBounds, Color.white, 15.0f);

                            if (localBounds.Contains(interpolatedPos)) {
                                isCenterFree = false;
                                break;
                            }
                        }
                    }

                    if (isCenterFree) {
                        Vector2 gridPos = Node.RoundVector2(new Vector2(interpolatedPos.x, interpolatedPos.z));

                        if (!graph.ContainsKey(gridPos)) {
                            Node node = new Node(gridPos);
                            graph.Add(gridPos, node);        
                        }
                    }
                }
            }
            Vector3 start_pos = mapManager.localStartPosition;
            Vector3 goal_pos = mapManager.localGoalPosition;
            Vector2 startGridPos = new Vector2(start_pos.x, start_pos.z);
            Vector2 goalGridPos = new Vector2(goal_pos.x, goal_pos.z);

            if (!graph.ContainsKey(startGridPos)) {
                Node node = new Node(startGridPos);
                graph.Add(startGridPos, node);
            }
            if (!graph.ContainsKey(goalGridPos)) {
                Node node = new Node(goalGridPos);
                graph.Add(goalGridPos, node);
            }  

                
                
            // ********  Dealing with the depart object  *********
            GameObject specialObject = GameObject.Find("startOverhead");
            if(specialObject != null) {
                
                Bounds worldBound = specialObject.GetComponent<Renderer>().bounds;
                Bounds localBound = new Bounds(
                    obstacleMap.grid.WorldToLocal(worldBound.center), 
                    obstacleMap.grid.WorldToLocal(worldBound.size)
                );
                float halfSize = 0.01f;
                for (float x = localBound.min.x; x < localBound.max.x; x += 0.1f) {
                    for (float z = localBound.min.z; z < localBound.max.z; z += 0.1f) {
                        // Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z + halfSize)), mapManager.grid.LocalToWorld(new Vector3(x+halfSize, 0, z-halfSize)), Color.red, 10000f);
                        // Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z -halfSize)), mapManager.grid.LocalToWorld(new Vector3(x +halfSize, 0, z +halfSize)), Color.red,10000f);
                        Vector3 pointInside = new Vector3(x, 0, z);


                        Collider[] hitColliders = Physics.OverlapSphere(mapManager.grid.LocalToWorld(pointInside), 0.1f);
                        if (hitColliders.Length == 1)
                        {
                            Vector3 posLocal = (new Vector3(Mathf.Floor(x / girdSizeX) * girdSizeX  + girdSizeX/2 , 0, Mathf.Floor(z / girdSizeZ) * girdSizeZ + girdSizeZ/2));
                            Vector2 gridPosS = Node.RoundVector2(new Vector2(posLocal.x , posLocal.z));
                            if (!graph.ContainsKey(gridPosS)) {
                                //Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(posLocal.x -halfSize, 0, posLocal.z + halfSize)), mapManager.grid.LocalToWorld(new Vector3(posLocal.x+halfSize, 0, posLocal.z-halfSize)), Color.red, 10000f);
                                //Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(posLocal.x -halfSize, 0, posLocal.z -halfSize)), mapManager.grid.LocalToWorld(new Vector3(posLocal.x +halfSize, 0, posLocal.z +halfSize)), Color.red,10000f);
                                Node node = new Node(gridPosS);
                                graph.Add(gridPosS, node);        
                            }
                            
                        }                    
                    }
                }
            }

            List<GameObject> obstacleObjects = mapManager.GetObstacleMap().obstacleObjects;

            foreach (GameObject obj in obstacleObjects) {
                if(!obj.name.Contains("startOverhead") && !obj.name.Contains("goalOverhead") && !obj.name.StartsWith("road_")){
                    Bounds worldBound = obj.GetComponent<Renderer>().bounds;
                    Bounds localBund = new Bounds(
                        obstacleMap.grid.WorldToLocal(worldBound.center), 
                        obstacleMap.grid.WorldToLocal(new Vector3(worldBound.size.x+3.3f, worldBound.size.y+3.3f, worldBound.size.z+3.3f))
                    );
                    // DrawBounds(localBund, Color.white, 15.0f);
                    float halfSize = 0.01f;
                    for (float x = localBund.min.x; x <= localBund.max.x+girdSizeX; x += girdSizeX) {
                        for (float z = localBund.min.z; z <= localBund.max.z+girdSizeX; z += girdSizeZ) {
                            // Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z + halfSize)), mapManager.grid.LocalToWorld(new Vector3(x+halfSize, 0, z-halfSize)), Color.red, 10000f);
                            // Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z -halfSize)), mapManager.grid.LocalToWorld(new Vector3(x +halfSize, 0, z +halfSize)), Color.red,10000f);
                            Vector3 pointInside = new Vector3(x, 0, z);
                            Vector3 posLocal = (new Vector3(Mathf.Floor(x / girdSizeX) * girdSizeX  + girdSizeX/2 , 0, Mathf.Floor(z / girdSizeZ) * girdSizeZ + girdSizeZ/2));
                            if(posLocal.x<=localBund.max.x && posLocal.x>=localBund.min.x && posLocal.z<=localBund.max.z && posLocal.z>=localBund.min.z){  
                                Vector2 gridPosS = Node.RoundVector2(new Vector2(posLocal.x , posLocal.z));
                                // Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(posLocal.x -halfSize, 0, posLocal.z + halfSize)), mapManager.grid.LocalToWorld(new Vector3(posLocal.x+halfSize, 0, posLocal.z-halfSize)), Color.blue, 10000f);
                                // Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(posLocal.x -halfSize, 0, posLocal.z -halfSize)), mapManager.grid.LocalToWorld(new Vector3(posLocal.x +halfSize, 0, posLocal.z +halfSize)), Color.blue,10000f);
                            
                                if (graph.ContainsKey(gridPosS)) {
                                    graph.Remove(gridPosS);   
                                }
                            }
                        }
                    }
                }
            }
        }

        private void DrawBounds(Bounds bounds, Color color, float delay)
        {
            Vector3 v3FrontTopLeft = mapManager.grid.LocalToWorld(new Vector3(bounds.min.x, bounds.max.y, bounds.min.z));
            Vector3 v3FrontTopRight = mapManager.grid.LocalToWorld(new Vector3(bounds.max.x, bounds.max.y, bounds.min.z));
            Vector3 v3FrontBottomLeft = mapManager.grid.LocalToWorld(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z));
            Vector3 v3FrontBottomRight = mapManager.grid.LocalToWorld(new Vector3(bounds.max.x, bounds.min.y, bounds.min.z));
            Vector3 v3BackTopLeft = mapManager.grid.LocalToWorld(new Vector3(bounds.min.x, bounds.max.y, bounds.max.z));
            Vector3 v3BackTopRight = mapManager.grid.LocalToWorld(new Vector3(bounds.max.x, bounds.max.y, bounds.max.z));
            Vector3 v3BackBottomLeft = mapManager.grid.LocalToWorld(new Vector3(bounds.min.x, bounds.min.y, bounds.max.z));
            Vector3 v3BackBottomRight = mapManager.grid.LocalToWorld(new Vector3(bounds.max.x, bounds.min.y, bounds.max.z));


            Debug.DrawLine(v3FrontTopLeft, v3FrontTopRight, color, delay);
            Debug.DrawLine(v3FrontTopRight, v3FrontBottomRight, color, delay);
            Debug.DrawLine(v3FrontBottomRight, v3FrontBottomLeft, color, delay);
            Debug.DrawLine(v3FrontBottomLeft, v3FrontTopLeft, color, delay);

            Debug.DrawLine(v3BackTopLeft, v3BackTopRight, color, delay);
            Debug.DrawLine(v3BackTopRight, v3BackBottomRight, color, delay);
            Debug.DrawLine(v3BackBottomRight, v3BackBottomLeft, color, delay);
            Debug.DrawLine(v3BackBottomLeft, v3BackTopLeft, color, delay);

            Debug.DrawLine(v3FrontTopLeft, v3BackTopLeft, color, delay);
            Debug.DrawLine(v3FrontTopRight, v3BackTopRight, color, delay);
            Debug.DrawLine(v3FrontBottomRight, v3BackBottomRight, color, delay);
            Debug.DrawLine(v3FrontBottomLeft, v3BackBottomLeft, color, delay);
        }

        public void ConnectToClosestNodes(Vector3 start, Vector3 goal, MapManager mapManager) {
            Node closestNodeToStart = FindClosestNode(start);
            Node closestNodeToGoal = FindClosestNode(goal);

            Vector3 closestNodeWorldPos = mapManager.grid.LocalToWorld(new Vector3(closestNodeToStart.GridPosition.x, 0, closestNodeToStart.GridPosition.y));
            Vector3 startWorldPos = mapManager.grid.LocalToWorld(new Vector3(start.x, 0, start.z));

            Vector2 startGridPos = new Vector2(start.x, start.z);
            Vector2 closeStartGridPos = new Vector2(closestNodeToStart.GridPosition.x, closestNodeToStart.GridPosition.y);
            if (!graph.ContainsKey(startGridPos)) {
                graph.Add(startGridPos, new Node(startGridPos));
            }  
            Node startnode = graph[startGridPos];     
            startnode.Neighbors.Add(graph[closeStartGridPos]); 

            Debug.DrawLine(startWorldPos, closestNodeWorldPos, Color.black, 70000f);

            Vector3 closestNodeWorldPosG = mapManager.grid.LocalToWorld(new Vector3(closestNodeToGoal.GridPosition.x, 0, closestNodeToGoal.GridPosition.y));
            Vector3 goalWorldPos = mapManager.grid.LocalToWorld(new Vector3(goal.x, 0, goal.z));

            Vector2 goalGridPos = new Vector2(goal.x, goal.z);
            Vector2 closeGoalGridPos = new Vector2(closestNodeToGoal.GridPosition.x, closestNodeToGoal.GridPosition.y);
            if (!graph.ContainsKey(goalGridPos)) {
                graph.Add(goalGridPos, new Node(goalGridPos));
            }
            Node goalNode = graph[goalGridPos];
            goalNode.Neighbors.Add(graph[closeGoalGridPos]);
            
            Debug.DrawLine(goalWorldPos, closestNodeWorldPosG, Color.black, 70000f);

        }

        private Node FindClosestNode(Vector3 point) {
            Node closestNode = null;
            Vector2 closestNodePos = new Vector2(point.x, point.z);
            float minDistance = float.MaxValue;
            float distance =float.MaxValue;

            foreach (var node in graph.Values) {
                    distance = Vector2.Distance(closestNodePos, node.GridPosition);
                    if (distance < minDistance && distance != 0) {
                        minDistance = distance;
                        closestNode = node;
                    }
            }
            return closestNode;
        }

        public void ConnectNodes(MapManager mapManager) 
        {

            float gridCellSize = mapManager.grid.cellSize.x;
            float girdSizeY = mapManager.grid.cellSize.y;
            float girdSizeZ = mapManager.grid.cellSize.z;
            Vector2[] directions = {
                new Vector2(gridCellSize, 0),
                new Vector2(-gridCellSize, 0),
                new Vector2(0, gridCellSize),
                new Vector2(0, -gridCellSize),
                new Vector2(gridCellSize, gridCellSize),
                new Vector2(-gridCellSize, gridCellSize),
                new Vector2(-gridCellSize, -gridCellSize),
                new Vector2(gridCellSize, -gridCellSize)
            };

            foreach (var grap in graph) {

                    // key is the current node's Vector2 position, same as node.gridPosition
                    Vector2 key = grap.Key;
                    Node node = grap.Value;
                    Dictionary<Vector2, Node> graphRef = graph;
                    // current node's World position
                    Vector3 nodeWorldPos = mapManager.grid.LocalToWorld(new Vector3(key.x, 0, key.y));

                    foreach (var dir in directions) { 
                        Vector2 neighborPos = key + dir;
                        Vector2 neighborPos_round = Node.RoundVector2(neighborPos);

                    
                        if (graph.ContainsKey(neighborPos_round)) {
                            Vector3 neighborLocalPos_V3 = new Vector3 (neighborPos_round.x, 0, neighborPos_round.y);
                            Vector3 neighborWorldPos =  mapManager.grid.LocalToWorld(neighborLocalPos_V3);
                            // Perform a raycast to check if the line of sight is clear
                            if (IsLineOfSightClear(nodeWorldPos, neighborWorldPos, mapManager)) {   
                                if((IsDiagonal(dir) ? AreIntermediateNodesPresent(key, neighborPos) : true)) {
                                    // Debug.DrawLine(nodeWorldPos, neighborWorldPos, Color.green, 70000f);
                                    node.Neighbors.Add(graph[neighborPos_round]);
                                }
                            }
                        }
                    }
            }
            ConnectToClosestNodes(mapManager.localStartPosition, mapManager.localGoalPosition, mapManager);
        }

        private bool IsDiagonal(Vector2 dir) {
            return Mathf.Abs(dir.x) == Mathf.Abs(dir.y);
        }

        private bool AreIntermediateNodesPresent(Vector2 start, Vector2 end) {
            Vector2 intermediate1 =  Node.RoundVector2(new Vector2(start.x, end.y));
            Vector2 intermediate2 =  Node.RoundVector2(new Vector2(end.x, start.y));

            return graph.ContainsKey(intermediate1) && graph.ContainsKey(intermediate2);
        }

            private bool IsLineOfSightClear(Vector3 start, Vector3 end, MapManager mapManager) 
            {
                RaycastHit hit;
                Vector3 direction = end - start;
                float distance = direction.magnitude;

                if (Physics.Raycast(start, direction.normalized, out hit, distance)) 
                {
                    if (mapManager.GetObstacleMap().obstacleObjects.Contains(hit.collider.gameObject)) 
                    {
                        return false;
                    }
                }
                return true;
            }

        void OnDrawGizmos() {
                Gizmos.color = Color.black;
                ObstacleMap obstacleMap = mapManager.GetObstacleMap();

                float girdSizeX = mapManager.grid.cellSize.x;
                float girdSizeY = mapManager.grid.cellSize.y;
                float girdSizeZ = mapManager.grid.cellSize.z;

                foreach (var posThreeDim in obstacleMap.mapBounds.allPositionsWithin)
                {
                    Vector3 localPosition = mapManager.grid.LocalToWorld(new Vector3(posThreeDim.x * girdSizeX + girdSizeX/2, posThreeDim.y * girdSizeY, posThreeDim.z * girdSizeZ + girdSizeZ/2));
                }
            }

        public List<Node> Bfs(Vector2 startGridPos, Vector2 goalGridPos){
            Debug.Log("**********  BFS  ********************");
            
            Node startNode = FindClosestNode(new Vector3(startGridPos.x, 0, startGridPos.y));
            Node goalNode = FindClosestNode(new Vector3(goalGridPos.x, 0, goalGridPos.y));

            Queue<Node> queue = new Queue<Node>();
            HashSet<Node> visited = new HashSet<Node>();
            Dictionary<Node, Node> parentMap = new Dictionary<Node, Node>();

            queue.Enqueue(startNode);
            visited.Add(startNode);
            Node current = startNode;

            while (queue.Count > 0) {
                current = queue.Dequeue();
                
                if (current == goalNode) {
                    Debug.Log("Goal reached");
                    break;
                }
                
                foreach (Node neighbor in current.Neighbors) {
                    if (!visited.Contains(neighbor)) {
                        // Visualize the path in real-time
                        Vector3 fromPos = new Vector3(current.GridPosition.x, 0, current.GridPosition.y);
                        Vector3 toPos = new Vector3(neighbor.GridPosition.x, 0, neighbor.GridPosition.y);
                        // Debug.DrawLine(mapManager.grid.LocalToWorld(fromPos), mapManager.grid.LocalToWorld(toPos), Color.cyan, 10000f);

                        visited.Add(neighbor);
                        parentMap[neighbor] = current; 
                        queue.Enqueue(neighbor);
                    }
                }
            }

            List<Node> path = new List<Node>();
            current = goalNode;

            while (current != null) {
                path.Add(current);
                current = parentMap.ContainsKey(current) ? parentMap[current] : null;
            }
            path.Reverse();

            // Debug.Log("Path Length: " + path.Count);
            return path;
        }



        private bool IsLineOfSightClear_Post(Vector3 start, Vector3 end, MapManager mapManager) 
        {
            RaycastHit hit;
            Vector3 direction = end - start;
            float distance = direction.magnitude + 80f;

            if (Physics.Raycast(start, direction.normalized, out hit, distance)) 
            {
                if (mapManager.GetObstacleMap().obstacleObjects.Contains(hit.collider.gameObject)) 
                {
                    if(hit.collider.gameObject.name.Contains("startOverhead") || hit.collider.gameObject.name.Contains("goal") || hit.collider.gameObject.name.StartsWith("road_")){
                        return true;
                    }
                    else {
                        //Debug.DrawLine(start, end, Color.cyan);
                        return false;
                    }
                }
            }
            return true;
        }

        private bool IsLineCrossingObjectBounds(Vector3 start, Vector3 end, MapManager mapManager) 
        {
            ObstacleMap obstacleMap = mapManager.GetObstacleMap();
            var allObjects = obstacleMap.obstacleObjects;
            var line = new Ray(start, end - start);
            float lineLength = Vector3.Distance(start, end);

            foreach (var obj in allObjects) 
            {
                if(!(obj.name.Contains("startOverhead") || obj.name.Contains("goal") || obj.name.StartsWith("road_"))){
                Bounds worldBound = obj.GetComponent<Renderer>().bounds;
                Bounds localBound = new Bounds(
                    obstacleMap.grid.WorldToLocal(worldBound.center), 
                    obstacleMap.grid.WorldToLocal(new Vector3(worldBound.size.x+3f, worldBound.size.y+3f, worldBound.size.z+3f))
                );
                // DrawBounds(localBound, Color.red, 15.0f);
                
                    if (obj.GetComponent<Collider>() != null)
                    {
                        if (localBound.IntersectRay(line, out float distance)) 
                        {
                            if (distance <= lineLength) 
                            {
                                return true; // Line intersects with the object's bounds
                            }
                        }
                    }
                }
            }

            return false; // No intersection with any object's bounds
        }

        public List<Vector3> AStarPost(List<Vector3> myPath, MapManager mapManager)
        {
            int k = 0;
            List<Vector3> t = new List<Vector3>();
            t.Add(myPath[0]);
            //t.Add(my_path[1]);

                for (int i = 1; i < myPath.Count - 1; i++){
                    //if (IsLineOfSightClear_Post(mapManager.grid.LocalToWorld(t[k]), mapManager.grid.LocalToWorld(myPath[i + 1]), mapManager) == false)
                    if (IsLineCrossingObjectBounds(t[k], myPath[i + 1], mapManager))
                    {
                        Debug.Log("hit");
                        k++;
                        t.Add(myPath[i]);
                        //break;
                    }
                }

            k++;
            
            t.Add(myPath[myPath.Count - 1]);
            return t;
        }

        private List<Vector3> InterpolatePath(List<Vector3> nodes, int pointsBetween)
        {
            List<Vector3> path = new List<Vector3>();
            for (int i = 0; i < nodes.Count - 1; i++)
            {
                Vector3 startNode = nodes[i];
                Vector3 endNode = nodes[i + 1];

                path.Add(startNode);

                for (int j = 1; j <= pointsBetween; j++)
                {
                    float t = j / (float)(pointsBetween + 1);
                    Vector3 interpolatedPoint = Vector3.Lerp(startNode, endNode, t);
                    path.Add(interpolatedPoint);
                }
            }
            path.Add(nodes[nodes.Count - 1]); // Add the last node

            return path;
        }
        



        private float Heuristic(Node current, Node goal) {
            float cost = Vector2.Distance(current.GridPosition, goal.GridPosition);
            // if (current.Neighbors.Count < 8) {
            //     cost *= 2;
            // }
            return cost;
        }

        private IEnumerable<State> GenerateSuccessorStates(State currentState) {
            var successors = new List<State>();

            foreach (var neighbor in currentState.CurrentNode.Neighbors) {
                float newCost = currentState.Cost + Node.Distance(currentState.CurrentNode, neighbor);
                successors.Add(new State(neighbor, newCost, currentState));
            }

            return successors;
        }

        public List<Node> AStarSearch(Vector2 startGridPos, Vector2 goalGridPos) {
            Debug.Log("**********  A*  ********************");

            StatePriorityQueue openSet = new StatePriorityQueue();
            HashSet<Node> closedSet = new HashSet<Node>();

            Node startNode = FindClosestNode(new Vector3(startGridPos.x, 0, startGridPos.y));
            Node goalNode = FindClosestNode(new Vector3(goalGridPos.x, 0, goalGridPos.y));

            State startState = new State(startNode, 0, null);
            startState.TotalEstimate = startState.Cost + Heuristic(startNode, goalNode);
            openSet.Add(startState);
            State currentState = null;
            State goalState=null;
            while (openSet.Count > 0) {
                currentState = openSet.Pop();

                if (currentState.CurrentNode == goalNode) {
                    goalState = currentState;
                    break;
                }

                if (closedSet.Contains(currentState.CurrentNode)) continue;
                closedSet.Add(currentState.CurrentNode);

                foreach (var nextState in GenerateSuccessorStates(currentState)) {
                    nextState.TotalEstimate = nextState.Cost + Heuristic(nextState.CurrentNode, goalNode);
                    if (!closedSet.Contains(nextState.CurrentNode)) {
                        openSet.Add(nextState);
                    }
                }
            }

            List<Node> path = new List<Node>();
            while (goalState != null) {
                path.Add(goalState.CurrentNode);
                goalState = goalState.Parent;
            }

            path.Reverse();
            Debug.Log("Path Length: " + path.Count);
            return path;
            
        }

        List<Vector3> CreateSmoothedPath(List<Vector3> originalPath)
        {
            List<Vector3> smoothedPath = new List<Vector3>();

            // Assuming each segment consists of 4 points from the original path
            for (int i = 0; i < originalPath.Count - 3; i += 3)
            {
                Vector3 p0 = originalPath[i];
                Vector3 p1 = originalPath[i + 1];
                Vector3 p2 = originalPath[i + 2];
                Vector3 p3 = originalPath[i + 3];

                // Sample points from the Bezier curve
                for (float t = 0; t <= 1; t += 0.05f) // Adjust the 0.05f for more or fewer points
                {
                    Vector3 point = CalculateCubicBezierPoint(t, p0, p1, p2, p3);
                    smoothedPath.Add(point);
                }
            }

            return smoothedPath;
        }

        Vector3 CalculateCubicBezierPoint(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            float u = 1 - t;
            float tt = t * t;
            float uu = u * u;
            float uuu = uu * u;
            float ttt = tt * t;

            Vector3 point = uuu * p0; // first term
            point += 3 * uu * t * p1; // second term
            point += 3 * u * tt * p2; // third term
            point += ttt * p3; // fourth term

            return point;
        }


        public List<Vector3>  BreakPoint(List<Vector3> nodes, float t)
        {

            List<Vector3> path = new List<Vector3>();
            for (int i = 0; i < nodes.Count - 1; i++)
            {
                Vector3 startNode = nodes[i];
                Vector3 endNode = nodes[i + 1];
                path.Add(startNode);

                Vector3 C = startNode + t * (endNode - startNode);
                path.Add(C);
                float x = C.x;
                float z = C.z;
                float halfSize = 0.01f;
                Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z + halfSize)), mapManager.grid.LocalToWorld(new Vector3(x+halfSize, 0, z-halfSize)), Color.red, 10000f);
                Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z -halfSize)), mapManager.grid.LocalToWorld(new Vector3(x +halfSize, 0, z +halfSize)), Color.red,10000f);

            }
            path.Add(nodes[nodes.Count - 1]); // Add the last node
            path.Add(nodes[nodes.Count - 1]); // Add the last node
            path.Add(nodes[nodes.Count - 1]); // Add the last node

            return path;
        }
        private void Start()
        {
            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            // get the car controller
            m_Car = GetComponent<CarController>();
            my_rigidbody = GetComponent<Rigidbody>();

            mapManager = FindObjectOfType<GameManager>().mapManager;

            Vector3 start_pos = mapManager.localStartPosition;
            Vector3 goal_pos = mapManager.localGoalPosition;

            targetPos = mapManager.grid.LocalToWorld(start_pos);

            InitializeGraph(mapManager);    
            
            ConnectNodes(mapManager);
            // foreach (Node node in graph.Values) {
            //         float halfSize = 0.01f;
            //         float x = node.GridPosition.x;
            //         float z = node.GridPosition.y;
            //         Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z + halfSize)), mapManager.grid.LocalToWorld(new Vector3(x+halfSize, 0, z-halfSize)), Color.red, 10000f);
            //         Debug.DrawLine( mapManager.grid.LocalToWorld(new Vector3(x -halfSize, 0, z -halfSize)), mapManager.grid.LocalToWorld(new Vector3(x +halfSize, 0, z +halfSize)), Color.red,10000f);
            // }
            
            Vector2 startGridPos = new Vector2(start_pos.x, start_pos.z);
            Vector2 goalGridPos = new Vector2(goal_pos.x, goal_pos.z);
            
            // List<Node> path = Bfs(startGridPos, goalGridPos);    
            List<Node> path = AStarSearch(startGridPos, goalGridPos);

            for (int i = 0; i < path.Count - 1; i++) {
                Vector3 fromPos = new Vector3(path[i].GridPosition.x, 0, path[i].GridPosition.y);
                Vector3 toPos = new Vector3(path[i + 1].GridPosition.x, 0, path[i + 1].GridPosition.y);
                //Debug.DrawLine(fromPos, toPos, Color.red, 10000f);
                Vector3 worldPos = fromPos; 
                my_path.Add(worldPos);
            }

            my_path.Add(goal_pos); 

            my_path = AStarPost(my_path, mapManager);

            my_path2 = BreakPoint(my_path, 0.70f);

            my_path = InterpolatePath(my_path, 10);

            // my_path = CreateSmoothedPath(my_path);

            // Plot your path to see if it makes sense
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.blue, 1000f);
                old_wp = wp;
            }
        }

        private int targetIndex = 0;
        private int targetIndex2 = 0;

        // for safe version Kp = 2, Kd = 1
        float k_p = 2f;
        float k_d = 1f; 
        private Vector3 targetPos;
        private Vector3 Old_targetPos;

        private void FixedUpdate(){
            Vector3 carPos_World = transform.position;
            Vector3 target_position00 = mapManager.grid.LocalToWorld(my_path2[targetIndex2]);

            if(Vector3.Distance(carPos_World, target_position00) < 8f) {
                if(targetIndex2 < my_path2.Count - 2){
                    targetIndex2 += 1;

                }
            } 
        
            Vector3 target_position1 = mapManager.grid.LocalToWorld(my_path[targetIndex]);

            if(Vector3.Distance(carPos_World, target_position1) < 7f) {
                if(targetIndex < my_path.Count - 1){
                    targetIndex += 1;
                    Old_targetPos = targetPos;
                }
            }
                    
            Vector3 targetPos_local = my_path[targetIndex];
            targetPos = mapManager.grid.LocalToWorld(targetPos_local);
            Debug.DrawLine(carPos_World, targetPos, Color.yellow);

            Vector3 targetVelocity = 0.05f * (targetPos - Old_targetPos)/Time.fixedDeltaTime;
            
            Vector3 position_error = (targetPos - carPos_World)*0.1f ;
            Vector3 velocity_error = (targetVelocity - my_rigidbody.velocity)*0.1f ;
            
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
            acceleration = Mathf.Clamp(acceleration, -1f, 1f);
           
            steering = Mathf.Clamp(steering, -1f, 1f);

            Debug.DrawLine(targetPos, targetPos + targetVelocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            float Velo_direction = Vector3.Dot(my_rigidbody.velocity.normalized, transform.forward);

            Vector3 BA = my_path2[targetIndex2 -1 ] - my_path2[targetIndex2];
            Vector3 BC = my_path2[targetIndex2 + 1] - my_path2[targetIndex2];

            float angle = Vector3.Angle(BA, BC);
            if (angle < 160 && Vector3.Dot(transform.forward, my_rigidbody.velocity) > 5f )
            {
                acceleration *= -0.1f;
                float frain = 0f;
                if(Vector3.Dot(transform.forward, my_rigidbody.velocity) > 20f){
                    frain = 1f;
                }
                else if(Vector3.Dot(transform.forward, my_rigidbody.velocity) > 15f){
                    frain = 0.5f;
                }
                m_Car.Move(steering, acceleration, acceleration, frain);
            }
            else{
                if(acceleration < 0 && Velo_direction < 0) {
                    m_Car.Move(-steering, 0, acceleration, 0f);
                }
                else if (acceleration > 0 && Velo_direction < 0){
                    m_Car.Move(-steering, acceleration, 0, 0);
                }
                else{
                    m_Car.Move(steering, acceleration, 0, 0f);
                }

            }
            Debug.Log("Steering : " + steering +"Angle: " + angle + "Accel: " +  Vector3.Dot(transform.forward, my_rigidbody.velocity));
        }
    }
}
