using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Map;
using Scripts.Vehicle;
using UnityEngine;

using System.Linq;
using System.Collections.Generic;
using Scripts.Game;
using System.IO;
using MAS2024.Assignment2;
using System.Runtime.Serialization.Formatters;
using System;

[RequireComponent(typeof(DroneController))]
public class AIP4FormationDrone : MonoBehaviour
{
    private DroneController m_Drone;

    public static bool allPass = false;
    public static List<bool> isReady = new List<bool> { false, false, false, false, false };
    public static int currentGateIdx = 0;
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private LineOfSightGoal m_CurrentGoal;

    public float steering;
    public float acceleration;

    // public readonly float CELLSIZE = 0.25f;

    private List<Vector2> positions;
    private List<Vector2> velocities;
    private List<float> times;

    private Vector2 prevPos;

    public readonly float k_p = 2f;
    public readonly float k_d = 1.5f;

    public GameObject my_target;

    private int target_pos_vel_idx = 0;
    private CollisionDetector detector;

    private Vector3 target_position;
    private Vector3 target_velocity;
    private Vector3 old_target_pos;
    private float MARGIN = 1.5f;
    private float startTime;
    private float timeLimit = 10f; // Stop searching after a certain time

    private static bool hasExecuted = false;
    private static int curCarIdx = 0;
    private int thisCarIdx;

    private static List<int> carIdxOrder = new List<int> {2, 4, 0, 3, 1};
    private void Start()
    {   
        // See AIP2TrafficDrone.cs for more examples
        thisCarIdx = curCarIdx;
        curCarIdx++;
        m_Drone = GetComponent<DroneController>();
        m_MapManager = FindObjectOfType<MapManager>();
        m_ObstacleMapManager = FindObjectOfType<ObstacleMapManager>();
        m_ObstacleMap = m_ObstacleMapManager.ObstacleMap; //Is not a MonoBehavior, so cannot fetch in the same fashion!

        var sw = System.Diagnostics.Stopwatch.StartNew();
        sw.Stop();
        Debug.Log($"Detector: {sw.ElapsedMilliseconds} ms");

        m_OtherCars = GameObject.FindGameObjectsWithTag("Player");

        sw.Restart();
        string posFilePath = Path.Combine(Application.persistentDataPath, $"car-positions-{m_MapManager.fileName}-{thisCarIdx}.json");
        string timesFilePath = Path.Combine(Application.persistentDataPath, $"car-times-{m_MapManager.fileName}-{thisCarIdx}.json");
        // naivePath = NaivePathPlanning();
        // // if(thisCarIdx == 0){
        // //     naivePath = NaivePathPlanning();
        // // }
        // return;

        // // For finding individual gates
        var gateGroup = m_MapManager.GetTargetObjects();
        detector = new CollisionDetector(m_MapManager, m_ObstacleMap, margin: MARGIN, yMax: 1.0f);
        Debug.Log($"My position is: {transform.position}");
        Vector3 startPos;
        Vector3 targetPosition;
        Vector3 startVel;
        RRT rrt;
        int gateIdx = carIdxOrder[thisCarIdx];
        // rrt = initRRT();
        // startPos = transform.position;
        // targetPosition = gateGroup[0].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center - gateGroup[1].transform.forward * 3f;
        // startVel = targetPosition - startPos;
        // Debug.Log("This car is at: " + thisCarIdx);
        // (positions, times) = rrt.FindPath(startPos, targetPosition, startVel, detector, timeLimit);
        // rrt.DebugDrawPath();
        // Debug.Log($"GateGroup: {gateGroup.Count}");
        // for(int i = 0; i < gateGroup.Count - 1; i++)
        // {
        //     rrt = initRRT();
        //     startPos = gateGroup[i].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center - gateGroup[i].transform.forward * 5f;
        //     targetPosition = gateGroup[i+1].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center - gateGroup[i + 1].transform.forward * 5f;
        //     startVel = gateGroup[i].transform.forward;
        //     Debug.DrawLine(startPos, startPos + startVel * 10f, Color.red, 100f);
        //     List<Vector2> followingPositions;
        //     List<float> followingTimes;
        //     Debug.Log($"Gate {i}: {gateGroup[i]}");
        //     Debug.Log($"Gate {i} StartPos: {startPos}");
        //     Debug.Log($"Gate {i} TargetPos: {targetPosition}");
        //     Debug.DrawLine(startPos, targetPosition, Color.red, 100f);
        //     Debug.DrawLine(startPos, startPos + Vector3.up * 10, Color.blue, 100f);
        //     Debug.DrawLine(targetPosition, targetPosition + Vector3.up * 10, Color.blue, 100f);
        //     (followingPositions, followingTimes) = rrt.FindPath(startPos, targetPosition, startVel, detector, timeLimit); // List<Vector2> of positions, List<float> of timestamps
        //     rrt.DebugDrawPath();
        //     TrackingUtil.concatenatePositionsAndTimes(ref positions, ref times, followingPositions, followingTimes);
        // }        // times.Add(times[times.Count - 1] + 2f);
        // string posJson = JsonUtility.ToJson(new SerializableList<Vector2>(positions));
        // string timesJson = JsonUtility.ToJson(new SerializableList<float>(times));
        // File.WriteAllText(posFilePath, posJson);
        // File.WriteAllText(timesFilePath, timesJson);
        // Debug.Log($"Save file to: position => {posFilePath}, times =>{timesFilePath}");


        SerializableList<Vector2> posData = JsonUtility.FromJson<SerializableList<Vector2>>(File.ReadAllText(posFilePath));
        positions = posData.list;
        SerializableList<float> timesData = JsonUtility.FromJson<SerializableList<float>>(File.ReadAllText(timesFilePath));
        times = timesData.list;
        positions.Add(Vector3ToVector2(gateGroup[gateGroup.Count - 1].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center + gateGroup[gateGroup.Count - 1].transform.forward * 1f));
        times.Add(times[times.Count - 1] + 1f);
        positions.Add(Vector3ToVector2(gateGroup[gateGroup.Count - 1].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center + gateGroup[gateGroup.Count - 1].transform.forward * 2f));
        times.Add(times[times.Count - 1] + 1.5f);
        positions.Add(Vector3ToVector2(gateGroup[gateGroup.Count - 1].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center + gateGroup[gateGroup.Count - 1].transform.forward * 3f));
        times.Add(times[times.Count - 1] + 2f);

        //Debug draw the last point in positions
        Debug.DrawLine(Vector2ToVector3(positions[positions.Count - 1]), Vector2ToVector3(positions[positions.Count - 1]) + Vector3.up * 10, Color.cyan, 100f);


        // Draw all the positions in the list
        for (int i = 0; i < positions.Count - 1; i++)
        {
            Debug.DrawLine(Vector2ToVector3(positions[i]), Vector2ToVector3(positions[i + 1]), Color.red, 100f);
        }
        startTime = Time.realtimeSinceStartup;
    }


    private void FixedUpdate()
    {
        // NaiveControl();
        ControlCar();
        // if (Time.realtimeSinceStartup - startTime < 3f){
        //     m_Drone.Move(0f, 1f, 1f, 0f);
        // }
        // else if (Time.realtimeSinceStartup - startTime < 10f){
        //     m_Drone.Move(0f, -1f, 0f, 1f);
        // }
        // else{
        //     m_Drone.Move(0f, 1f, 1f, 0f);
        // }
    }
     private void OnDrawGizmos()
    {
        // detector.DebugDrawBoundingBoxes();
        //detector.DebugDrawGrid();
    }
    private Vector2 Vector3ToVector2(Vector3 v)
    {
        return new Vector2(v.x, v.z);
    }
    private Vector3 Vector2ToVector3(Vector2 v)
    {
        return new Vector3(v.x, 0.4f, v.y);
    }


    Vector3 rushDir;
    private void ControlCar()
    {   
        Vector2 dronePos = new Vector2(transform.position.x, transform.position.z);
        Vector2 currentVelocity = Vector3ToVector2(GetComponent<Rigidbody>().velocity);
        float speed = currentVelocity.magnitude;
        prevPos = dronePos;

        // Look ahead proportional to speed
        float baseLookAhead = 5f;
        float referenceSpeed = 10f;
        float lookAhead = baseLookAhead * Mathf.Max(speed / referenceSpeed, 0.5f);
        baseLookAhead = 5f;
        // Get position and velocity that far ahead on the path
        Vector2 lookAheadPos, lookAheadVel;
        (lookAheadPos, lookAheadVel) = TrackingUtil.LookAheadPositionAndVelocity(dronePos, lookAhead, positions, times);

        // Calculate parallel and transverse velocity components wrt the direction of the look ahead position
        Vector2 lookDir = (lookAheadPos - dronePos).normalized;
        float velocityParallel = Vector2.Dot(currentVelocity, lookDir);
        Vector2 velocityTransverse = currentVelocity - velocityParallel * lookDir; // Use this to kill velocity in any direction other than the one we want to go

        // Calculate inputs
        float ap = TrackingUtil.CalculateAccelerationUnclamped(dronePos, lookAheadPos, velocityParallel, lookAheadVel.magnitude);
        float balanceMultiplier = 2f;
        Vector2 accel = ap * lookDir - velocityTransverse * balanceMultiplier;

        Debug.DrawLine(Vector2ToVector3(dronePos), Vector2ToVector3(lookAheadPos), Color.magenta);
        // m_Drone.Move(accel.x, accel.y);
        // ================== Control the car ==================
        
        var gateGroup = m_MapManager.GetTargetObjects();
        if (currentGateIdx >= gateGroup.Count){
            m_Drone.Move(accel.x, accel.y);
            return;
        }
        Vector3 gatePos3D = gateGroup[currentGateIdx].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center;

        Debug.DrawLine(Vector2ToVector3(dronePos), Vector2ToVector3(dronePos) + Vector3.up * 10f, Color.blue);
        Debug.DrawLine(gatePos3D, gatePos3D + Vector3.up * 10f, Color.red);

        Vector3 gatePos2D = Vector3ToVector2(gatePos3D);
        Debug.Log($"Car {thisCarIdx} is at {dronePos}, Gate {currentGateIdx} is at {gatePos2D} Distance: {Vector2.Distance(dronePos, gatePos2D)}");
        if (Vector2.Distance(dronePos, gatePos2D) < 10f)
        {   
            Debug.Log("Close to gate");
            // Formation behavior
            if (!isReady.All(b => b))
            {
                Debug.Log($"Car {thisCarIdx} velocity {GetComponent<Rigidbody>().velocity.magnitude}");
                if (GetComponent<Rigidbody>().velocity.magnitude < 0.1f)
                {
                    isReady[thisCarIdx] = true;
                    m_Drone.Move(0, 0);
                    rushDir = gatePos3D - transform.position;
                }
                else
                {
                    Debug.Log($"This car idx: {thisCarIdx}, Ready? {isReady[thisCarIdx]}");
                    m_Drone.Move(-currentVelocity.x, -currentVelocity.y);
                }
            }
            else if (isReady.All(b => b))
            {   
                m_Drone.Move(rushDir.x, rushDir.z);
                if (thisCarIdx == 2 && Vector2.Dot(Vector3ToVector2(GetComponent<Rigidbody>().velocity), Vector3ToVector2(gatePos3D - transform.position)) < 0)
                {
                    Debug.LogError($"Update");
                    currentGateIdx = currentGateIdx + 1;
                    isReady = new List<bool> { false, false, false, false, false };
                }
                Debug.Log($"This car idx: {thisCarIdx}, All Ready!");
            }
        }
        else
        {   
            // Normal driving behavior
            Debug.Log($"This car idx: {thisCarIdx}, {steering}, {acceleration}");
            m_Drone.Move(accel.x, accel.y);
        }

        for(int i = 0; i < m_OtherCars.Length; i++)
        {
            if (i == thisCarIdx)
            {
                continue;
            }
            // Vector2 orientation = Vector3ToVector2(m_OtherCars[i].transform.forward);
            // Vector2 toOther = Vector3ToVector2(transform.position - m_OtherCars[i].transform.position);
            float angleToOthers = Vector2.Dot(Vector3ToVector2(GetComponent<Rigidbody>().velocity), Vector3ToVector2(m_OtherCars[i].transform.position - transform.position));
            Debug.Log($"Car {thisCarIdx} is at {dronePos}, Car {i} is at {Vector3ToVector2(m_OtherCars[i].transform.position)} Angle: {angleToOthers} Distance: {Vector2.Distance(dronePos, Vector3ToVector2(m_OtherCars[i].transform.position))}");
            if(angleToOthers > 0.1 && Vector2.Distance(dronePos, Vector3ToVector2(m_OtherCars[i].transform.position)) < 5f)
            {   
                Debug.Log($"Car {thisCarIdx} is too close to car {i}");
                m_Drone.Move(-currentVelocity.x, -currentVelocity.y);
            }
        }



    }

private List<Vector3> NaivePathPlanning(){
        // Connect the middle gate between each group
        // Make interpolation between the gates
        List<Vector3> naivePath = new List<Vector3>();
        int nMiddlePoints;
        var gateGroup = m_MapManager.GetTargetObjects();
        int middlePointFactor = 10;
        nMiddlePoints = middlePointFactor * (int) Mathf.Round(Vector3.Distance(transform.position, gateGroup[0].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center));
        for(int j = 0; j < nMiddlePoints; j++){
            Vector3 middlePos = Vector3.Lerp(transform.position, gateGroup[0].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center, (float)j / nMiddlePoints);
            naivePath.Add(middlePos);
        }  
        for(int i = 0; i < gateGroup.Count - 1; i++){
            Vector3 startPos = gateGroup[i].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center + gateGroup[i].transform.forward * 0f;
            Vector3 endPos = gateGroup[i+1].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center - gateGroup[i+1].transform.forward * 0f;
            // Make interpolations
            nMiddlePoints = middlePointFactor * (int) Mathf.Round(Vector3.Distance(startPos, endPos));
            for(int j = 0; j < nMiddlePoints; j++){
                Vector3 middlePos = Vector3.Lerp(startPos, endPos, (float)j / nMiddlePoints);
                naivePath.Add(middlePos);
            }
            startPos = gateGroup[i+1].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center - gateGroup[i+1].transform.forward * 0f;
            endPos = gateGroup[i+1].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center + gateGroup[i+1].transform.forward * 0f;
            nMiddlePoints = middlePointFactor * (int) Mathf.Round(Vector3.Distance(startPos, endPos));  
            for(int j = 0; j < nMiddlePoints; j++){
                Vector3 middlePos = Vector3.Lerp(startPos, endPos, (float)j / nMiddlePoints);
                naivePath.Add(middlePos);
            }                  
        }

        //Debug draw the path
        for(int i = 0; i < naivePath.Count - 1; i++){
            Debug.DrawLine(naivePath[i], naivePath[i+1], Color.green, 100f);
        }

        naivePrePos = naivePath[index];

        return naivePath;
    }

    private Vector3 getRelativePos(){
        if (thisCarIdx == 0){
            return Vector3.zero;
        }
        else{
            Vector3 thisGateRElativePos = m_MapManager.GetTargetObjects()[currentGateIdx].transform.GetChild(carIdxOrder[0]).GetComponent<MeshCollider>().bounds.center - m_MapManager.GetTargetObjects()[currentGateIdx].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center;
            return m_OtherCars[0].transform.position - thisGateRElativePos;
        }
    }

    private Vector3 naivePrePos;
    int index = 1;
    float oldTime = 0f;
    List<Vector3> naivePath;
    private void NaiveControl(){


        Vector3 currentPos = transform.position;
        Vector3 targetPos;
        Vector3 targetVel;
        float lookAhead = 2f;
        float referenceSpeed = 20f;
        lookAhead = lookAhead * (referenceSpeed / GetComponent<Rigidbody>().velocity.magnitude );
        float minDistance = 4f;

        float updateIterval;
        if(GetComponent<Rigidbody>().velocity.magnitude > 3f){
            updateIterval = 0.0020f;
        }
        else{
            updateIterval = 0.00020f;
        }

        if (thisCarIdx == 0){
            if (Time.realtimeSinceStartup - oldTime > updateIterval){
                index++;
                oldTime = Time.realtimeSinceStartup;
                Debug.Log($"Index: {index}, oldTime: {oldTime}");
            }
            targetPos = naivePath[index];
            Debug.DrawLine(targetPos, targetPos + Vector3.up * 10f, Color.yellow);
            Vector3 curGoalPos = m_MapManager.GetTargetObjects()[currentGateIdx].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center + m_MapManager.GetTargetObjects()[currentGateIdx].transform.forward * 5f;
            Debug.DrawLine(curGoalPos, curGoalPos + Vector3.up * 10f, Color.blue);
            if(Vector3.Distance(transform.position, curGoalPos) < 4.5f){
                currentGateIdx++;   
            }            
        }
        else{
            // Draw the relative position
            if (thisCarIdx == 1){
            Debug.Log($"Dot {Vector3.Dot(getRelativePos() - transform.position, transform.forward)}");
            }
            if (Vector3.Dot(getRelativePos() - transform.position, m_MapManager.GetTargetObjects()[currentGateIdx].transform.forward) > 0f){
                targetPos = getRelativePos();
                //targetPos = Vector3.zero;
            }
            else{
                targetPos = Vector3.zero;
            }
            Debug.DrawLine(targetPos, 10f * Vector3.up + targetPos, Color.red);
        }
        if (Time.realtimeSinceStartup - oldTime > updateIterval){
            index++;
            oldTime = Time.realtimeSinceStartup;
            Debug.Log($"Index: {index}, oldTime: {oldTime}");
        }
        // targetPos = naivePath[index];
        // Debug.DrawLine(targetPos, targetPos + Vector3.up * 10f, Color.yellow);
        // Vector3 curGoalPos = m_MapManager.GetTargetObjects()[currentGateIdx].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center + m_MapManager.GetTargetObjects()[currentGateIdx].transform.forward * 5f;
        // Debug.DrawLine(curGoalPos, curGoalPos + Vector3.up * 10f, Color.blue);
        // if(Vector3.Distance(transform.position, curGoalPos) < 4.5f){
        //     currentGateIdx++;   
        // } 

        if (targetPos == Vector3.zero){
            Debug.Log($"Car {thisCarIdx} breaking");
            m_Drone.Move(-GetComponent<Rigidbody>().velocity.x, -GetComponent<Rigidbody>().velocity.z);
            return;
        }

        // Debug.Log($"Car {thisCarIdx} is at {currentPos}, Target is at {targetPos} Distance: {Vector3.Distance(currentPos, targetPos)}");
        // Debug.Log(Vector3.Distance(currentPos, naivePath[index]) < minDistance);

        // Debug.DrawLine(targetPos, targetPos + Vector3.up * 10f, Color.green);

        targetVel = (targetPos - naivePrePos) / Time.fixedDeltaTime;
        naivePrePos = targetPos;
        Vector3 position_error = targetPos - transform.position;
        Vector3 velocity_error = targetVel - GetComponent<Rigidbody>().velocity;
        Vector3 desired_acceleration = 0.5f * position_error + 0.5f * velocity_error;

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
        steering = Mathf.Clamp(steering, -1f, 1f);
        Debug.Log($"position_error: {position_error}, velocity_error: {velocity_error}, desired_acceleration: {desired_acceleration}, steering: {steering}, acceleration: {acceleration}");
        Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        Debug.DrawLine(transform.position, transform.position + GetComponent<Rigidbody>().velocity, Color.blue);
        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);
        Debug.DrawLine(transform.position, transform.position + position_error, Color.magenta);

        // this is how you control the car
        // m_Drone.Move(steering, acceleration, Math.Max(-0.5f, acceleration), 0f);
        m_Drone.Move(steering, acceleration);
        // if(GetComponent<Rigidbody>().velocity.magnitude > 5f && thisCarIdx == 0){
        //     m_Drone.Move(steering, 0f, 0f, 0f);
        // }
        // else{
        //     m_Drone.Move(steering, acceleration, acceleration, 0f);
        // }
    }

    private RRT initRRT(){
        RRT rrt = new RRT();

        // Set the dynamic constraints
        rrt.localPlanner.acceleration = 6f;
        rrt.localPlanner.deceleration = 6f;
        rrt.localPlanner.minSpeed = 15f;
        rrt.localPlanner.maxSpeed = 25f;
        rrt.localPlanner.minTurnRadius = 5f;
        rrt.localPlanner.turnRadiusUncap = 15f;

        // Set RRT settings
        rrt.areaSearch = true;
        rrt.pruning = false;
        rrt.startWithKnn = true;
        rrt.goalPathBias = true;
        rrt.biasProbability = 0.5f;
        rrt.biasRadius = 5f;
        rrt.goalDistance = 2f;
        return rrt;
    }
}