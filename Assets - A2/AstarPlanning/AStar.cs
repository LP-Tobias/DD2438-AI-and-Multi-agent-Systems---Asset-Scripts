using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Scripts.Map;

namespace AstarPlanning
{
    public class AStar
    {
        private Vector3 startPos;
        private Vector3 targetPos;
        
        private Dictionary<Vector2, Node> graph;
        
        private List<Vector3> my_path = new List<Vector3>(); 
        
        private MapManager m_MapManager;
        private ObstacleMapManager m_ObstacleMapManager;
        private ObstacleMap m_ObstacleMap;
        
        float girdSizeX;
        float girdSizeY;
        float girdSizeZ;
        
        // Constructor
        public AStar(Vector3 startPos, Vector3 targetPos, MapManager mapManager, ObstacleMapManager obstacleMapManager) 
        {
            // eliminate the initial y 
            // the input is in world coordinates
            this.startPos = new Vector3(startPos.x, 0, startPos.z);
            this.targetPos = new Vector3(targetPos.x, 0, targetPos.z);
            m_MapManager = mapManager;
            m_ObstacleMapManager = obstacleMapManager;
            m_ObstacleMap = m_ObstacleMapManager.ObstacleMap;
            
            girdSizeX = m_MapManager.grid.cellSize.x;
            girdSizeY = m_MapManager.grid.cellSize.y;
            girdSizeZ = m_MapManager.grid.cellSize.z;
        }
        
        /* Generate the graph of the map */
        public void InitializeGraph() 
        {
            graph = new Dictionary<Vector2, Node>();
            
            // Iterates over CELL positions in the map.  "(x, z, 0)" or (x, y) in 2nd coordinates
            foreach (var posThreeDim in m_ObstacleMap.cellBounds.allPositionsWithin) {
                // Debug.Log("Pos Three Dim: " + posThreeDim);
                Vector2Int posTwoDim = new Vector2Int((int)posThreeDim.x, (int)posThreeDim.y);
                ObstacleMap.Traversability status = m_ObstacleMap.IsCellTraversable(posTwoDim);

                if (status != ObstacleMap.Traversability.Blocked ) {
                    // Debug.Log("not blocked");
                    // cellCenter in Vector3
                    Vector3 cellCenterLocal = m_ObstacleMap.mapGrid.GetCellCenterLocal(posThreeDim);
                    Vector3 cellCenterWorld = m_ObstacleMap.mapGrid.GetCellCenterWorld(posThreeDim);
                    
                    //Debug.Log("Cell Center: " + cellCenter + " Cell Center Local: " + cellCenterLocal + " Cell Center World: " + cellCenterWorld);
                    
                    List<GameObject> objectsInCell = m_ObstacleMap.gameGameObjectsPerCell[posTwoDim];
                    List<GameObject> allObjects = m_ObstacleMap.obstacleObjects;

                    bool isCenterFree = true;
                    foreach (GameObject obj in allObjects) {
                        if (obj.name.Contains("start") || obj.name.Contains("goal"))
                        {
                            continue;
                        }
                    
                        // Convert the world bounds of the object to local bounds
                        Bounds worldBounds = obj.GetComponent<Renderer>().bounds;
                        Bounds localBounds = new Bounds(
                            m_ObstacleMap.mapGrid.WorldToLocal(worldBounds.center), 
                            // cook: for highway, onramp, x append 10f. for intersection no append.
                            m_ObstacleMap.mapGrid.WorldToLocal(new Vector3(worldBounds.size.x, worldBounds.size.y+4f, worldBounds.size.z))
                        );
                        //DrawBounds(localBounds, Color.white, 15.0f);
                        
                        if (localBounds.Contains(cellCenterLocal)) { 
                            isCenterFree = false;
                            //if(obj.name.Contains("start")) { isCenterFree = true;}
                            break; // The cell center is occupied, no need to check other objects
                        }
                    }

                    if (isCenterFree) {
                        // What should I store here, local or world? Maybe local...
                        Vector2 gridPos = Node.RoundVector2(new Vector2(cellCenterLocal.x, cellCenterLocal.z));
                        // Debug.Log("Append to graph: " + gridPos);
                        
                        if (!graph.ContainsKey(gridPos)) {
                            Node node = new Node(gridPos);
                            graph.Add(gridPos, node);        
                        }
                    }
                }
            }
            
            // Add the start and target position, local
            Vector3 startLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(startPos);
            Vector3 targetLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(targetPos);
            Vector2 startGridPos = new Vector2(startLocalPos.x, startLocalPos.z);
            Vector2 targetGridPos = new Vector2(targetLocalPos.x, targetLocalPos.z);
            
            if (!graph.ContainsKey(startGridPos)) {
                Node node = new Node(startGridPos);
                graph.Add(startGridPos, node);
            }
            if (!graph.ContainsKey(targetGridPos)) {
                Node node = new Node(targetGridPos);
                graph.Add(targetGridPos, node);
            }

            //test first and see if special object deal is needed.
            
        }
        
        public void ConnectNodes()
        {
            float gridCellSize = girdSizeX;
             
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
                
                // Current node's World position
                Vector3 nodeWorldPos = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(key.x, 0, key.y));
                
                foreach (var dir in directions) { 
                    
                    Vector2 neighborPos = key + dir;
                    Vector2 neighborPos_round = Node.RoundVector2(neighborPos);
                    
                    if (graph.ContainsKey(neighborPos_round)) {
                        Vector3 neighborLocalPos_V3 = new Vector3 (neighborPos_round.x, 0, neighborPos_round.y);
                        Vector3 neighborWorldPos =  m_ObstacleMap.mapGrid.LocalToWorld(neighborLocalPos_V3);
                        // Perform a rayCast to check if the line of sight is clear
                        if (IsLineOfSightClear(nodeWorldPos, neighborWorldPos)) {   
                            if((IsDiagonal(dir) ? AreIntermediateNodesPresent(key, neighborPos) : true)) {
                                // Debug.DrawLine(nodeWorldPos, neighborWorldPos, Color.green, 70000f);
                                node.Neighbors.Add(graph[neighborPos_round]);
                            }
                        }
                    }
                }
            }
            
            // Connect the start and target nodes to the closest nodes
            ConnectToClosestNodes();
        }
        
        private bool IsLineOfSightClear(Vector3 start, Vector3 end) 
        {
            RaycastHit hit;
            Vector3 direction = end - start;
            float distance = direction.magnitude;

            if (Physics.Raycast(start, direction.normalized, out hit, distance)) 
            {
                if (m_ObstacleMap.obstacleObjects.Contains(hit.collider.gameObject)) 
                {
                    return false;
                }
            }
            return true;
        }
        
        private bool IsDiagonal(Vector2 dir) {
            return Mathf.Abs(dir.x) == Mathf.Abs(dir.y);
        }
        
        private bool AreIntermediateNodesPresent(Vector2 start, Vector2 end) {
            Vector2 intermediate1 =  Node.RoundVector2(new Vector2(start.x, end.y));
            Vector2 intermediate2 =  Node.RoundVector2(new Vector2(end.x, start.y));

            return graph.ContainsKey(intermediate1) && graph.ContainsKey(intermediate2);
        }
        
        public void ConnectToClosestNodes() {
            // For finding the closest node need to convert the world position to local position
            Vector3 startLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(startPos);
            Vector3 targetLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(targetPos);
            Node closestNodeToStart = FindClosestNode(startLocalPos);
            Node closestNodeToGoal = FindClosestNode(targetLocalPos);

            Vector3 closestNodeWorldPos = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(closestNodeToStart.GridPosition.x, 0, closestNodeToStart.GridPosition.y));

            Vector2 startGridPos = new Vector2(startLocalPos.x, startLocalPos.z);
            Vector2 closeStartGridPos = new Vector2(closestNodeToStart.GridPosition.x, closestNodeToStart.GridPosition.y);
            if (!graph.ContainsKey(startGridPos)) {
                graph.Add(startGridPos, new Node(startGridPos));
            }  
            Node startnode = graph[startGridPos];     
            startnode.Neighbors.Add(graph[closeStartGridPos]); 
            
            //Debug.Log("Start Node: " + startPos + " Closest Node: " + closestNodeWorldPos);
            // Debug.DrawLine(startPos, closestNodeWorldPos, Color.black, 70000f);

            Vector3 closestNodeWorldPosG = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(closestNodeToGoal.GridPosition.x, 0, closestNodeToGoal.GridPosition.y));

            Vector2 goalGridPos = new Vector2(targetLocalPos.x, targetLocalPos.z);
            Vector2 closeGoalGridPos = new Vector2(closestNodeToGoal.GridPosition.x, closestNodeToGoal.GridPosition.y);
            if (!graph.ContainsKey(goalGridPos)) {
                graph.Add(goalGridPos, new Node(goalGridPos));
            }
            Node goalNode = graph[goalGridPos];
            goalNode.Neighbors.Add(graph[closeGoalGridPos]);
            
            // Debug.DrawLine(targetPos, closestNodeWorldPosG, Color.white, 70000f);
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
        
        /* A* algorithm */
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
        
        public void AStarSearch() {
            // Debug.Log("****  A*  ********");
            Vector3 startLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(startPos);
            Vector3 targetLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(targetPos);
            Vector2 startGridPos = new Vector2(startLocalPos.x, startLocalPos.z);
            Vector2 goalGridPos = new Vector2(targetLocalPos.x, targetLocalPos.z);

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
            // Debug.Log("Path Length: " + path.Count);
            
            for (int i = 0; i < path.Count - 1; i++) {
                Vector3 fromPos = new Vector3(path[i].GridPosition.x, 0, path[i].GridPosition.y);
                Vector3 toPos = new Vector3(path[i + 1].GridPosition.x, 0, path[i + 1].GridPosition.y);
                // Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(fromPos), m_ObstacleMap.mapGrid.LocalToWorld(toPos), Color.yellow, 10000f); 
                my_path.Add(fromPos);
            }
            my_path.Add(targetLocalPos);
        }
        
        private float Heuristic(Node current, Node goal) {
            float cost = Vector2.Distance(current.GridPosition, goal.GridPosition);
            // check if the current node has all the 8 neighbors if not add a cost of *2 to the heuristic
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
        
        /* A* postprocessing*/
        public List<Vector3> AStarPost()
        {
            // returning the path list of vector3
            int k = 0;
            List<Vector3> t = new List<Vector3>();
            t.Add(my_path[0]);

            for (int i = 1; i < my_path.Count - 1; i++){
                if (IsLineCrossingObjectBounds(t[k], my_path[i + 1]))
                {
                    k++;
                    t.Add(my_path[i]);
                }
            }
            k++;
            
            t.Add(my_path[my_path.Count - 1]);
            return t;
        }
        
        private bool IsLineCrossingObjectBounds(Vector3 start, Vector3 end) 
        {
            var allObjects = m_ObstacleMap.obstacleObjects;
            var line = new Ray(start, end - start);
            float lineLength = Vector3.Distance(start, end);

            foreach (var obj in allObjects) 
            {
                if(!(obj.name.Contains("start") || obj.name.Contains("goal"))){
                    Bounds worldBound = obj.GetComponent<Renderer>().bounds;
                    Bounds localBound = new Bounds(
                        m_ObstacleMap.mapGrid.WorldToLocal(worldBound.center), 
                        m_ObstacleMap.mapGrid.WorldToLocal(new Vector3(worldBound.size.x, worldBound.size.y, worldBound.size.z))
                    );
                    // DrawBounds(localBound, Color.white, 15.0f);
                    
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
        
        private void DrawBounds(Bounds bounds, Color color, float delay)
        {
            Vector3 v3FrontTopLeft = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.min.x, bounds.max.y, bounds.min.z));
            Vector3 v3FrontTopRight = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.max.x, bounds.max.y, bounds.min.z));
            Vector3 v3FrontBottomLeft = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z));
            Vector3 v3FrontBottomRight = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.max.x, bounds.min.y, bounds.min.z));
            Vector3 v3BackTopLeft = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.min.x, bounds.max.y, bounds.max.z));
            Vector3 v3BackTopRight = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.max.x, bounds.max.y, bounds.max.z));
            Vector3 v3BackBottomLeft = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.min.x, bounds.min.y, bounds.max.z));
            Vector3 v3BackBottomRight = m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(bounds.max.x, bounds.min.y, bounds.max.z));


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
        
    }
}