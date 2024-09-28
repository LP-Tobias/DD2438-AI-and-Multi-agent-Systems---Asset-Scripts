using System;
using System.Linq;
using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;
using System.IO;
using MAS2024.Assignment2;
using ORCA;
using AstarPlanning;
using AVO;

[RequireComponent(typeof(CarController))]
public class AIP1TrafficCar : MonoBehaviour
{
    private CarController m_Car;
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private LineOfSightGoal m_CurrentGoal;
    private BoxCollider carCollider;
    private Rigidbody m_rigidbody;
    public List<GameObject> obstacleObjects;

    public AStar AStarPlanner;
    private List<Vector3> A_StarPath;
    
    private float tauObj = 5f;
    private float delta = 10f;

    public Vector3 startPos, targetPos;
    private Vector3 lookAtPos, previousPos;
    
    private void Start()
    {
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        m_rigidbody = GetComponent<Rigidbody>();
        m_Car = GetComponent<CarController>();
        m_CurrentGoal = (LineOfSightGoal)FindObjectOfType<GameManagerA2>().vehicleToGoalMapping[gameObject]; //This car's goal.
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!
        obstacleObjects = m_ObstacleMap.obstacleObjects;
        
        m_OtherCars = GameObject.FindGameObjectsWithTag("Player");
        
        startPos = gameObject.transform.position;
        targetPos = m_CurrentGoal.targetPosition; //World Coordinates.

        AStarPlanner = new AStar(startPos, targetPos, m_MapManager, m_ObstacleMapManager);
        AStarPlanner.InitializeGraph();
        AStarPlanner.ConnectNodes();
        AStarPlanner.AStarSearch();
        A_StarPath = AStarPlanner.AStarPost();
        
        // Plot the path
        Vector3 startLocalPos = m_ObstacleMap.mapGrid.WorldToLocal(startPos);
        Vector3 old_wp = startLocalPos;
        foreach (var wp in A_StarPath)
        {
            Debug.DrawLine(m_ObstacleMap.mapGrid.LocalToWorld(old_wp), m_ObstacleMap.mapGrid.LocalToWorld(wp), Color.white, 1000f);
            old_wp = wp;
        }
        
        // Interpolate the path
        A_StarPath = InterpolatePath(A_StarPath);
        
        // Mapping to world coordinates
        for(int i = 0; i < A_StarPath.Count; i++)
        {
            A_StarPath[i] = m_ObstacleMap.mapGrid.LocalToWorld(A_StarPath[i]);
        }
        
        // Initialize the previousPos
        previousPos = m_MapManager.GetGlobalStartPosition();

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
            
            // based on the se_dist calculate the number of points to add
            int pointsBetween = (int) (se_dist / 0.5f);

            for (int j = 1; j <= pointsBetween; j++)
            {
                float t = j / (float)(pointsBetween + 1);
                Vector3 interpolatedPoint = Vector3.Lerp(startNode, endNode, t);

                // Check if the distance to the next node is at least 8 units
                float dist = Vector3.Distance(m_ObstacleMap.mapGrid.LocalToWorld(interpolatedPoint), m_ObstacleMap.mapGrid.LocalToWorld(endNode));
                if (dist >= 0.5f)
                {
                    float x = interpolatedPoint.x;
                    float z = interpolatedPoint.z;
                    float halfSize = 0.01f;
                    Debug.DrawLine( m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x -halfSize, 0, z + halfSize)), m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x+halfSize, 0, z-halfSize)), Color.red, 10000f);
                    Debug.DrawLine( m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x -halfSize, 0, z -halfSize)), m_ObstacleMap.mapGrid.LocalToWorld(new Vector3(x +halfSize, 0, z +halfSize)), Color.red,10000f);

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
        m_OtherCars = GameObject.FindGameObjectsWithTag("Player");
        ControlCar_PD_Tracking();
    }
    
    private float radius = 5f;
    private float tau = 3f;
    public Boolean isTurn = false;

    private void ControlCar_PD_Tracking()
    {
        // Car behaviour
        float max_speed = m_Car.MaxSpeed; // 50
        float max_steering = m_Car.m_MaximumSteerAngle; // 25
        float max_acceleration = 20f; // 2
        Vector3 horizon = new Vector3(transform.right.x, 0, transform.right.z).normalized;
        Vector3 vertical = new Vector3(transform.forward.x, 0, transform.forward.z).normalized;

        // Current Position and Velocity
        Vector3 currentPos = transform.position;
        previousPos = currentPos;
        Vector3 currentVelo = new Vector3(m_rigidbody.velocity.x, 0, m_rigidbody.velocity.z);
        float carSpeed = currentVelo.magnitude;

        
        // Look ahead proportional to speed
        float baseLookAhead = 20f;
        float lookAhead = baseLookAhead * Mathf.Max(carSpeed / max_speed, 0.5f);
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
        
        // Velocity Scale and Stopping Behavior.
        Vector3 targetVelocity = new Vector3(0f, 0f, 0f);
        bool VO_switch = true;
        float Scale = 1f;
        foreach (var otherCar in m_OtherCars)
        {
            AIP1TrafficCar otherCarScript = otherCar.GetComponent<AIP1TrafficCar>();
            if (Vector3.Distance(currentPos, otherCar.transform.position) < 20f)
            {
                Scale++;
                if (this.gameObject == otherCar)
                {
                    continue;
                }
                Vector2 orientation = Vector3ToVector2(otherCar.transform.forward);
                Vector2 toOther = Vector3ToVector2(transform.position - otherCar.transform.position);
                float angleToOthers = Vector2.Dot(Vector3ToVector2(GetComponent<Rigidbody>().velocity), Vector3ToVector2(otherCar.transform.position - transform.position));
                if(angleToOthers > 0.5f && 
                   Vector2.Distance(Vector3ToVector2(currentPos), Vector3ToVector2(otherCar.transform.position)) < 8f )
                   // && Vector2.Dot(Vector3ToVector2(transform.forward), orientation) > 0f )
                {   
                    
                    VO_switch = false;
                    // Tricky bug. Finding solutions...
                    m_Car.Move(0f, 0f, 0f, 1f);
                    //return;
                }
                else
                {
                    VO_switch = true;
                }
            }
        }
        
        targetVelocity = Vector3.Normalize(lookAtPos - currentPos) * max_speed / Scale ;

        radius = Mathf.Max(3f * (carSpeed / max_speed), 2f);
        // Inserting VO for new target velocity
        VelocityObstacle vo = new VelocityObstacle(m_OtherCars, obstacleObjects,
            new Vector2(currentPos.x, currentPos.z),
            new Vector2(currentVelo.x, currentVelo.z),
            new Vector2(targetVelocity.x, targetVelocity.z),
            max_speed* Mathf.Max(1 / Scale, 0.3f), radius , 4f, 10f);
        vo.VO_Compute(false, false, false, 30f);
        Vector2 VO_Velocity = vo.newVelocity;
        if (VO_switch)
        {
            VO_Velocity = vo.newVelocity;
        }
        else
        {
            VO_Velocity = Vector3ToVector2(targetVelocity);
        }
        
        /* AVO implementation, but it is not as good. */
        // Accel_VO avo = new Accel_VO(m_OtherCars, obstacleObjects,
        //     new Vector2(currentPos.x, currentPos.z),
        //     new Vector2(currentVelo.x, currentVelo.z),
        //     new Vector2(targetVelocity.x, targetVelocity.z),
        //     2f, 5f, 1f, 0.001f, max_speed, 5f);
        // avo.AVO_compute();
        // Vector2 VO_Velocity = avo.newVelocity;
        
        Vector3 position_error = lookAtPos - currentPos;
        Vector3 velocity_error = new Vector3(VO_Velocity.x, 0, VO_Velocity.y) - currentVelo;

        Vector3 desired_acceleration;
        float steering;
        float acceleration;

        desired_acceleration = 0f * position_error + 1f * velocity_error;
        float angle = Vector2.SignedAngle(Vector3ToVector2(desired_acceleration), Vector3ToVector2(vertical));
        steering = Mathf.Clamp(angle / max_steering, -1, 1);
        acceleration = Mathf.Clamp(Vector3.Dot(desired_acceleration, vertical) / max_acceleration, -1, 1);
        
        float Velo_direction = Vector3.Dot(currentVelo.normalized, vertical);
        
        float VO_bias = Vector3.SignedAngle(lookAtPos - currentPos, new Vector3(VO_Velocity.x, 0, VO_Velocity.y), Vector3.up);
        float curVe_bias = Vector2.SignedAngle(Vector3ToVector2(lookAtPos - currentPos), Vector3ToVector2(currentVelo));
        
        var isComplete = m_CurrentGoal.IsAchieved();
        if (isComplete)
        {
            m_Car.Move(0f, 1f, 0f, 0f);
        }
        else
        {
            if (VO_bias < 5f || VO_bias > -5f || VO_bias > 175f || VO_bias < -175f)
            {
                if (acceleration < 0 && Velo_direction < 0)
                {
                    m_Car.Move(-steering, 0, acceleration, 0f);
                }
                else if (acceleration > 0 && Velo_direction < 0)
                {
                    m_Car.Move(-steering, acceleration, 0, 0);
                }
                else
                {
                    m_Car.Move(steering, acceleration, acceleration, 0f);
                }
            }
            else
            {
                m_Car.Move(0f, 0f, 0f, 1f);
            }
        }

        // m_Car.Move(steering, acceleration, acceleration, 0f);
        Debug.DrawLine(currentPos, lookAtPos, Color.yellow);
        Debug.DrawLine(transform.position, transform.position + currentVelo, Color.white);
        Debug.DrawLine(transform.position, transform.position + horizon * steering, Color.black);
        Debug.DrawLine(transform.position, transform.position + vertical * acceleration, Color.magenta);
        Debug.DrawLine(currentPos, currentPos + new Vector3(VO_Velocity.x, 0, VO_Velocity.y), Color.blue);
    }
    
}