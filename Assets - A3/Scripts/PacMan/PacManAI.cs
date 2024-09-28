using System;
using System.Collections.Generic;
using System.Linq;
using Scripts.Map;
using UnityEngine;

namespace PacMan
{
    public class PacManAI : MonoBehaviour
    {
        private IPacManAgent  agentManager;
        private ObstacleMapManager obstacleMap;
        private ObstacleMap _map;

        private int gridMinX, gridMinY, gridMaxX, gridMaxY;
        private float cellSize;

        private UtilityAgent utilityAgent;

        private Vector3 closestFood = new Vector3(-1, -1, -1);
        private Vector3 closestCapsule = new Vector3(-1, -1, -1);
        private Vector3 closestAllyCapsule = new Vector3(-1, -1, -1);
        private Vector3 closestAlly = new Vector3(-1, -1, -1);
        private Vector3 closestEnemy = new Vector3(-1, -1, -1);
        private MazeDistanceCalculator distanceCalculator;

        private static bool distancesCalculated = false;
        private static MazeDistanceCalculator staticDistanceCalculator;

        public int foodLeft;
        public int returnedFood = 0;

        private bool returnToBase = false;
        public Vector3 closestCenter = new Vector3(-1, -1, -1);

        public bool offensive = true;
        public bool red = true;

        PacManGameManager pmgManager;

        public void Initialize()
        {
            agentManager = GetComponent<IPacManAgent>();
            obstacleMap = FindObjectOfType<ObstacleMapManager>();
            pmgManager = FindObjectOfType<PacManGameManager>();
            obstacleMap.Start();
            _map = obstacleMap.ObstacleMap;

            gridMinX = _map.cellBounds.min.x; gridMinY = _map.cellBounds.min.y;
            gridMaxX = _map.cellBounds.max.x; gridMaxY = _map.cellBounds.max.y;
            cellSize = obstacleMap.grid.cellSize.x;

            // Calculate the distances only once
            if (!distancesCalculated)
            {
                staticDistanceCalculator = new MazeDistanceCalculator();
                staticDistanceCalculator.CalculateDistances(_map, cellSize, gridMinX, gridMaxX, gridMinY, gridMaxY);
                distancesCalculated = true;
                //Debug.Log("Calculated the distances by agent: " + agentManager.GetInstanceID());
            }

            // Use the static distance calculator for all agents
            distanceCalculator = staticDistanceCalculator;

            red = agentManager.GetStartPosition().x > 0;
            utilityAgent = new UtilityAgent(agentManager, _map, distanceCalculator, red);
        }

        private Vector3 GetClosestFood(Vector3 position)
        {
            //if (closestFood.x != -1 && closestFood.y != -1 && closestFood.z != -1 && Vector3.Distance(position, closestFood) > 0.5
            //    && !(distanceCalculator.GetDistance(position, closestFood) > 10 && distanceCalculator.GetDistance(position, closestFood) < 12))
            //{
            //    return closestFood;
            //}

            red = gameObject.CompareTag("Red");

                List<GameObject> foodPositions = red
                    ? agentManager.GetFoodObjects().Where(food => food.transform.position.x < 0).ToList()
                    : agentManager.GetFoodObjects().Where(food => food.transform.position.x > 0).ToList();

            foodLeft = foodPositions.Count;

            if (foodPositions.Count == 0)
                return Vector3.zero;

            // Get the positions of the closest food for all friendly agents
            List<Vector3> otherAgentsFoodPositions = new List<Vector3>();
            List<IPacManAgent> otherPacmans = agentManager.GetFriendlyAgents();
            foreach (var op in otherPacmans)
            {
                if (op == agentManager) continue;
                otherAgentsFoodPositions.Add(op.gameObject.GetComponent<PacManAI>().closestFood);
            }

            Vector3 closest = Vector3.zero;
            float ds = float.MaxValue;

            foreach (var food in foodPositions)
            {
                // Skip the food that is already the closest for another agent
                if (otherAgentsFoodPositions.Contains(food.transform.position))
                    continue;

                float curr_ds = distanceCalculator.GetDistance(food.transform.position, position);
                if (curr_ds < ds)
                {
                    ds = curr_ds;
                    closest = food.transform.position;
                }
            }

            // Debug.Log("returning closest food: " + closest);
            return closest;
        }

        private Vector3 GetClosestCapsule(Vector3 position)
        {
            //if (closestCapsule.x != -1 && closestCapsule.y != -1 && closestCapsule.z != -1 && Vector3.Distance(position, closestCapsule) > 0.5)
            //{
            //    return closestCapsule;
            //}
            List<GameObject> capsulesPositions = red
                ? agentManager.GetCapsuleObjects().Where(capsule => capsule.transform.position.x < 0).ToList()
                : agentManager.GetCapsuleObjects().Where(capsule => capsule.transform.position.x > 0).ToList();

            if (capsulesPositions.Count == 0)
                return Vector3.zero;

            Vector3 closest = capsulesPositions[0].transform.position;
            float dist = float.MaxValue;
            foreach (var capsule in capsulesPositions)
            {
                if (distanceCalculator.GetDistance(capsule.transform.position, position) < dist)
                {
                    closest = capsule.transform.position;
                    dist = distanceCalculator.GetDistance(closest, position);
                }
            }

            return closest;
        }
        private Vector3 GetClosestAllyCapsule(Vector3 position)
        {
            //if (closestCapsule.x != -1 && closestCapsule.y != -1 && closestCapsule.z != -1 && Vector3.Distance(position, closestCapsule) > 0.5)
            //{
            //    return closestCapsule;
            //}
            List<GameObject> capsulesPositions = red
                ? agentManager.GetCapsuleObjects().Where(capsule => capsule.transform.position.x > 0).ToList()
                : agentManager.GetCapsuleObjects().Where(capsule => capsule.transform.position.x < 0).ToList();

            if (capsulesPositions.Count == 0)
                return Vector3.zero;

            Vector3 closest = capsulesPositions[0].transform.position;
            float dist = float.MaxValue;
            foreach (var capsule in capsulesPositions)
            {
                if (distanceCalculator.GetDistance(capsule.transform.position, position) < dist)
                {
                    closest = capsule.transform.position;
                    dist = distanceCalculator.GetDistance(closest, position);
                }
            }

            return closest;
        }

        private Vector3 GetClosestAlly(Vector3 position)
        {
            List<IPacManAgent> otherPacmans = agentManager.GetFriendlyAgents();
            Vector3 closest = Vector3.zero;
            float dist = float.MaxValue;
            foreach (var op in otherPacmans)
            {
                if (op == agentManager) continue;
                if (distanceCalculator.GetDistance(op.gameObject.transform.position, position) < dist)
                {
                    closest = op.gameObject.transform.position;
                    dist = distanceCalculator.GetDistance(closest, position);
                }
            }
            return closest;
        }

        private Vector3 GetClosestEnemy(Vector3 position)
        {
            PacManObservations fetchEnemyObservation = agentManager.GetEnemyObservations();
            List<PacManObservation> fetchEnemyObservations = fetchEnemyObservation.Observations.Where(enemy => enemy.Visible==true).ToList();
            Vector3 closest = new(-1, -1, -1);
            float dist = float.MaxValue;

            foreach(var op in fetchEnemyObservations)
            {
                if(distanceCalculator.GetDistance(position, op.Position) < dist)
                {
                    closest = op.Position;
                    dist = distanceCalculator.GetDistance(position, closest);
                }
            }
            return closest; 
        }

        private Vector3 GetClosestCenter(Vector3 position)
        {
            if (closestCenter.x != 0 && closestCenter.y != 0 && closestCenter.z != 0)
            {
                if (Vector3.Distance(position, closestCenter) > 0.5) return closestCenter;

                closestCenter.x = 0;
                closestCenter.y = 0;
                closestCenter.z = 0;
                return closestCenter;
            }

            if (Vector3.Distance(position, closestCenter) > 3)             
                closestCenter = distanceCalculator.GetClosestCenterCell(position, red);
            return closestCenter; 
        }

        private int counter = 0;
        public PacManAction Tick()
        {
            if (true)
            {
                Vector3 currPos = transform.position;
                if(counter%10==0) closestFood = GetClosestFood(currPos);
                closestCapsule = GetClosestCapsule(currPos);
                closestAllyCapsule = GetClosestAllyCapsule(currPos);
                closestAlly = GetClosestAlly(currPos);
                closestEnemy = GetClosestEnemy(currPos); // returns (-1, -1, -1) if no enemy is visible
                closestCenter = GetClosestCenter(currPos); // should only be called when relevant
                bool isPacman = red ? currPos.x < 0 : currPos.x > 0;
                bool isScared = agentManager.IsScared();
                bool isEnemyScared = agentManager.IsPoweredUp();
                int carriedFood = agentManager.GetCarriedFoodCount();
                int scoreDifference = red ? pmgManager.redScore - pmgManager.blueScore : pmgManager.blueScore - pmgManager.redScore;
                counter++;

                if (closestFood == Vector3.zero && closestCapsule == Vector3.zero && counter % 50 == 0)
                {
                    closestCenter = distanceCalculator.GetClosestCenterCell(currPos, red);
                    //Debug.DrawLine(currPos, closestCenter, Color.cyan);
                }
                //Debug.DrawLine(currPos, closestFood, Color.white);
                //Debug.DrawLine(currPos, closestCapsule, Color.yellow);
                //Debug.DrawLine(currPos, closestAlly, Color.green);
                //if (closestEnemy.y != -1)
                //{
                //    Debug.DrawLine(currPos, closestEnemy, Color.red);
                //}

                //Debug.DrawLine(currPos, closestCenter, Color.gray);


                return utilityAgent.
                    ChooseAction(
                        currPos,
                        closestFood,
                        closestCapsule,
                        closestAllyCapsule,
                        closestAlly,
                        closestEnemy,
                        closestCenter,
                        isPacman,
                        isScared,
                        isEnemyScared,
                        carriedFood,
                        scoreDifference, 
                        red
                    );
            } else
            {
                //Debug.Log(agentManager.GetStartPosition());
                return new()
                {
                    AccelerationDirection = new(0, 0),
                    AccelerationMagnitude = 0,
                };
            }
        }
    }
}

//public PacManAction Tick() //The Tick from the network controller
//{
//    bool isGhost = agentManager.IsGhost();
//    bool isScared = agentManager.IsScared();
//    float scaredDuration = agentManager.GetScaredRemainingDuration();

//    float carriedFoodCount = agentManager.GetCarriedFoodCount();

//    List<GameObject> foodPositions = agentManager.GetFoodObjects();
//    List<GameObject> capsulePositions = agentManager.GetCapsuleObjects();

//    var isLocalPointTraversable = _map?.IsLocalPointTraversable(transform.localPosition);

//    var friendlyAgentManager = agentManager.GetFriendlyAgents()[0];
//    friendlyAgentManager.IsGhost();

//    var visibleEnemyAgents = agentManager.GetVisibleEnemyAgents();
//    Debug.Log(visibleEnemyAgents.Count);

//    List<PacManObservation> fetchEnemyObservations = agentManager.GetEnemyObservations();
//    if (fetchEnemyObservations?.Count > 0)
//    {
//        //  Debug.Log(fetchEnemyObservations[0].position + " :" + fetchEnemyObservations[0].readingDispersion); //Uniform random
//    }

//    // Since the RigidBody is updated server side and the client only syncs position, rigidbody.Velocity does not report a velocity
//    agentManager.GetVelocity(); // Use the manager method to get the true velocity from the server
//    // friendlyAgentManager.GetVelocity(); // Given the damping, max velocity magnitude is around 2.34

//    // // replace the human input below with some AI stuff
//    var x = UnityEngine.Random.Range(-1, 2);
//    var z = UnityEngine.Random.Range(-1, 2);

//    var droneAction = new PacManAction
//    {
//        AccelerationDirection = new Vector2(x, z), // Controller converts to [0; 1] normalized acceleration vector,
//        AccelerationMagnitude = 1f // 1 means max acceleration in chosen direction // 0.3 guarantees not observed
//    };

//    return droneAction;
//}