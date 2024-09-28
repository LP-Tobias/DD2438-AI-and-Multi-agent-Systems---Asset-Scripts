using System;
using Scripts.Vehicle;
using UnityEngine;
using System.Linq;
using System.Collections.Generic;
using Scripts.Game;
using Scripts.Map;
using MAS2024.Assignment2;
using ORCA;
using AstarPlanning;
using AVO;

/*
 * Before running check:
 * Cell size setting
 * Tau setting
 * Radius setting.
 */

[RequireComponent(typeof(DroneController))]
public class AIP2TrafficDrone : MonoBehaviour
{
    private DroneController m_Drone;
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherDrones;
    private LineOfSightGoal m_CurrentGoal;
    private BoxCollider droneCollider;
    private Rigidbody m_rigidbody;
    public List<GameObject> obstacleObjects;
    
    private List<Vector3> A_StarPath;

    private float radius = 1.5f;
    private float tau = 1f; 
    /*
     * Definition tau: to avoid VO that might hit in tau secs. So if tau is bigger then the drone will avoid the VO earlier.
     * if want a good visualization  set tau = 2
     * if want a good performance at highway / onramp / intersection, set tau small.
     */
    private float tauObj = 5f;
    
    // for acceleration vo
    private float delta = 2f;

    private Vector3 startPos, targetPos;
    
    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        m_rigidbody = m_Drone.GetComponent<Rigidbody>();
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject]; //This obj's goal.
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!
        obstacleObjects = m_ObstacleMap.obstacleObjects;
        
        m_OtherDrones = GameObject.FindGameObjectsWithTag("Player");
        
        startPos = gameObject.transform.position;
        targetPos = m_CurrentGoal.targetPosition; //World Coordinates.
        
        AStar AStarPlanner = new AStar(startPos, targetPos, m_MapManager, m_ObstacleMapManager);
        AStarPlanner.InitializeGraph();
        AStarPlanner.ConnectNodes();
        AStarPlanner.AStarSearch();
        A_StarPath = AStarPlanner.AStarPost();
        
        Vector3 startLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(startPos);
        Vector3 old_wp = startLocalPos;
        foreach (var wp in A_StarPath)
        {
            //Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp), Color.white, 1000f);
            old_wp = wp;
        }
        
        // Interpolate the path
        A_StarPath = InterpolatePath(A_StarPath);
        
        //For tracker
        for(int i = 0; i < A_StarPath.Count; i++)
        {
            A_StarPath[i] = m_ObstacleMap.mapGrid.LocalToWorld(A_StarPath[i]);
        }
        
        lookAtPos = m_ObstacleMap.mapGrid.LocalToWorld(A_StarPath[1]);
    }
    
    private List<Vector3> InterpolatePath(List<Vector3> nodes)
    {
        List<Vector3> path = new List<Vector3>();
        for (int i = 0; i < nodes.Count - 1; i++)
        {
            Vector3 startNode = nodes[i];
            Vector3 endNode = nodes[i + 1];
            float se_dist = Vector3.Distance(m_ObstacleMap.mapGrid.LocalToWorld(startNode), m_ObstacleMap.mapGrid.LocalToWorld(endNode));

            path.Add(startNode);

            int pointsBetween = (int) (se_dist / 5f);

            for (int j = 1; j <= pointsBetween; j++)
            {
                float t = j / (float)(pointsBetween + 1);
                Vector3 interpolatedPoint = Vector3.Lerp(startNode, endNode, t);

                // Check if the distance to the next node is at least 8 units
                float dist = Vector3.Distance(m_ObstacleMap.mapGrid.LocalToWorld(interpolatedPoint), m_ObstacleMap.mapGrid.LocalToWorld(endNode));
                if (dist >= 0.1f)
                {
                    float x = interpolatedPoint.x;
                    float z = interpolatedPoint.z;
                    float halfSize = 0.01f;
                    // Debug.DrawLine( m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x -halfSize, 0, z + halfSize)), m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x+halfSize, 0, z-halfSize)), Color.red, 10000f);
                    // Debug.DrawLine( m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x -halfSize, 0, z -halfSize)), m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x +halfSize, 0, z +halfSize)), Color.red,10000f);

                    path.Add(interpolatedPoint);

                }
            }
        }

        path.Add(nodes[nodes.Count - 1]); // Add the last node

        return path;
    }
    
    private Vector2 Vector3ToVector2(Vector3 v)
    {
        return new Vector2(v.x, v.z);
    }
    
    private Vector3 Vector2ToVector3(Vector2 v)
    {
        return new Vector3(v.x, 0.4f, v.y);
    }

    private void FixedUpdate()
    {
        m_OtherDrones = GameObject.FindGameObjectsWithTag("Player");
        ControlDrone_PD_Tracker();
    }
    
    private void ControlDrone_PD_Tracker()
    {     
        // Drone behaviors:
        Vector3 horizon = new Vector3(transform.right.x, 0, transform.right.z);
        Vector3 vertical = new Vector3(transform.forward.x, 0, transform.forward.z);
        float max_speed = m_Drone.max_speed;

        // get the current world Position and Velo
        Vector3 currentPos = new Vector3(transform.position.x, 0f, transform.position.z);
        Vector3 currentVelo = new Vector3(m_rigidbody.velocity.x, 0, m_rigidbody.velocity.z);
        float droneSpeed = currentVelo.magnitude;
        
        // Look ahead proportional to speed
        float baseLookAhead = 30f;
        float lookAhead = baseLookAhead * Mathf.Max(droneSpeed / max_speed, 0.5f);
        float minDistance = float.MaxValue;
        int index = 0;
        for (int i = 0; i < A_StarPath.Count; i++)
        {
            float distance = (currentPos - A_StarPath[i]).sqrMagnitude;
            
            if (distance < minDistance)
            {
                index = i;
                minDistance = distance;
            }
        }
        int lookAheadIndex = -1;
        for (int i = index; i < A_StarPath.Count; i++)
        {
            float distance = (currentPos - A_StarPath[i]).magnitude;
            if (distance > lookAhead)
            {
                lookAheadIndex = i - 1;
                break;
            }
        }
        if (lookAheadIndex == -1)
        {
            lookAheadIndex = A_StarPath.Count - 1;
        }
        lookAtPos = A_StarPath[lookAheadIndex];
        
        // calculate the turning angle
        Vector3 vecAhead = A_StarPath[lookAheadIndex + 1] - lookAtPos;
        Vector3 vecBack = A_StarPath[lookAheadIndex - 1] - lookAtPos;
        float turnAngle = Vector3.SignedAngle(vecAhead, vecBack, Vector3.up);
        Boolean isTurn = Mathf.Abs(turnAngle) != 180f;
        if (isTurn && Vector3.Distance(lookAtPos, currentPos) >= 3f)
        {
            lookAheadIndex = lookAheadIndex - 1;
            lookAtPos = A_StarPath[lookAheadIndex];
        }
        
        Vector3 targetVelocity = new Vector3 (0f, 0f, 0f);
        var isComplete = m_CurrentGoal.IsAchieved();
        if (!isComplete) {targetVelocity = Vector3.Normalize(lookAtPos - currentPos) * max_speed;}

        // Inserting VO for new target velocity
        VelocityObstacle vo = new VelocityObstacle(m_OtherDrones, obstacleObjects,
            new Vector2(currentPos.x, currentPos.z), 
            new Vector2(currentVelo.x, currentVelo.z), 
            new Vector2(targetVelocity.x, targetVelocity.z), 
            max_speed, 1f, 1.5f, 4f);
        vo.VO_Compute(false, false, true, -1f); // no static on highway
        Vector2 VO_Velocity = vo.newVelocity;
        
        Vector3 position_error = lookAtPos - currentPos;
        Vector3 velocity_error = new Vector3(VO_Velocity.x, 0, VO_Velocity.y) - currentVelo;
        
        // Debug.Log("targetVelo : " + targetVelocity + " VO_Velo: " + VO_Velocity + " myVelo: " + currentVelo);

        Vector3 desired_acceleration;
        float h;
        float v;
        
        desired_acceleration = 2f * position_error + 5f * velocity_error;
        h = Vector3.Dot(desired_acceleration, horizon) ;
        v = Vector3.Dot(desired_acceleration, vertical) ;
        
        if (isComplete)
        {
            m_Drone.Move(0f, 0f);
        }
        else
        {
            m_Drone.Move(h,v);
        }
        
        // Debug.DrawLine(currentPos, lookAtPos, Color.yellow);
        Debug.DrawLine(currentPos, currentPos + new Vector3(VO_Velocity.x, 0, VO_Velocity.y), Color.red);
        Debug.DrawLine(transform.position, transform.position + currentVelo, Color.white);
        // Debug.DrawLine(transform.position, transform.position + horizon * h, Color.black);
        // Debug.DrawLine(transform.position, transform.position + vertical* v, Color.magenta);
    }
    
    // ***************************************************
    // Generally just using PD_Tracker.
    // PD: in some terrain faster.
    // OpenTerrain: for testing ORCA in open terrain.
    
    private int lookAtIndex = 1;
    private Vector3 lookAtPos;
    private bool break_flag = false;
    private bool stop_flag = false;
    
    private void ControlDrone_PD()
    {     
        // Drone behaviors:
        Vector3 horizon = new Vector3(transform.right.x, 0, transform.right.z);
        Vector3 vertical = new Vector3(transform.forward.x, 0, transform.forward.z);
        float max_speed = m_Drone.max_speed;

        // get the current world Position and Velo
        Vector3 currentPos = new Vector3(transform.position.x, 0f, transform.position.z);
        Vector3 currentVelo = new Vector3(m_rigidbody.velocity.x, 0, m_rigidbody.velocity.z);
        
        // update the targetPosition
        if(Vector3.Distance(currentPos, lookAtPos) < 5f){
            lookAtPos = m_ObstacleMap.mapGrid.LocalToWorld(A_StarPath[lookAtIndex]);
            if (lookAtIndex < A_StarPath.Count - 1) { lookAtIndex++; }
            else { lookAtIndex = A_StarPath.Count - 1; }
        }
        
        // Debug.Log("lookAtPos: " + lookAtPos + " currentPos: " + currentPos + " distance: " + Vector3.Distance(currentPos, lookAtPos));
        
        if(lookAtIndex == A_StarPath.Count - 1){
            stop_flag = true;
        }
        
        Vector3 targetVelocity = new Vector3 (0f, 0f, 0f);

        // When reach the ending stick to it so don't mess around the others.
        if (stop_flag)
        {
            if (Vector3.Distance(currentPos, lookAtPos) > 4f) { targetVelocity = Vector3.Normalize(lookAtPos - currentPos) * 14f; }
            else { targetVelocity = new Vector3(0f, 0f, 0f); }
        }
        else
        {
            targetVelocity = Vector3.Normalize(lookAtPos - currentPos) * max_speed;
        }

        // Inserting VO for new target velocity
        VelocityObstacle vo = new VelocityObstacle(m_OtherDrones, obstacleObjects,
            new Vector2(currentPos.x, currentPos.z), 
            new Vector2(currentVelo.x, currentVelo.z), 
            new Vector2(targetVelocity.x, targetVelocity.z), 
            max_speed, 1f, 0.5f, 1f);
        vo.VO_Compute(false, false, false, 8f);
        Vector2 VO_Velocity = vo.newVelocity;
        
        Vector3 position_error = lookAtPos - currentPos;
        Vector3 velocity_error = new Vector3(VO_Velocity.x, 0, VO_Velocity.y) - currentVelo;
        
        // Debug.Log("targetVelo : " + targetVelocity + " VO_Velo: " + VO_Velocity + " myVelo: " + currentVelo);

        Vector3 desired_acceleration;
        float h;
        float v;
        
        desired_acceleration = 2f * position_error + 5f * velocity_error;
        h = Vector3.Dot(desired_acceleration, horizon) ;
        v = Vector3.Dot(desired_acceleration, vertical) ;
        
        m_Drone.Move(h,v);
        
        // Debug.DrawLine(currentPos, lookAtPos, Color.yellow);
        // Debug.DrawLine(currentPos, currentPos + new Vector3(VO_Velocity.x, 0, VO_Velocity.y), Color.red);
        Debug.DrawLine(transform.position, transform.position + currentVelo, Color.white);
        // Debug.DrawLine(transform.position, transform.position + horizon * h, Color.black);
        // Debug.DrawLine(transform.position, transform.position + vertical* v, Color.magenta);
    }
    
    private void ControlDrone_OpenTerrain()
    {
        Vector2 currentPos = new Vector2(transform.position.x, transform.position.z);
        Vector2 currentVelo_rigidbody = new Vector2(m_Drone.GetComponent<Rigidbody>().velocity.x, m_Drone.GetComponent<Rigidbody>().velocity.z);
        
        //for testing vo on open terrain
        Vector2 lineOfSight_err = Vector3ToVector2(targetPos) - currentPos;
        VelocityObstacle vo = new VelocityObstacle(m_OtherDrones, obstacleObjects, currentPos, currentVelo_rigidbody, lineOfSight_err, 15f, 10f, 2f, tauObj);
        vo.VO_Compute(false , false, false, -1f);
        Vector2 VO_Velocity = vo.newVelocity;
        
        //AVO is not working, Planning on make sth working using VO on cars then get back.
        // Accel_VO vo = new Accel_VO(m_OtherDrones, obstacleObjects, currentPos, currentVelo_rigidbody, lineOfSight_err, 
        //      4f, 5f, 1f, Time.deltaTime, 15f, 15f);
        // vo.AVO_compute();
        // Vector2 VO_Velocity = vo.newVelocity;
        // Debug.Log("VO_Velocity: " + VO_Velocity);
        

        Vector2 Velo_Err = VO_Velocity - currentVelo_rigidbody;
        
        Vector2 desired_accel = 0.5f * lineOfSight_err + 1f * Velo_Err;
        
        Vector2 horizon = new Vector2(transform.right.x, transform.right.z);
        Vector2 Vertical = new Vector2(transform.forward.x, transform.forward.z);
        m_Drone.Move(Vector2.Dot(desired_accel, horizon), Vector2.Dot(desired_accel, Vertical));
        
        // Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(currentPos + lineOfSight_err), Color.magenta);
        Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(currentPos + VO_Velocity), Color.blue);
    }
}