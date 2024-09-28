using System.Linq;
using System.Collections.Generic;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using Scripts.Game;
using Scripts.Map;
using UnityEngine;
using System.IO;
using MAS2024.Assignment2;
using System.Runtime.Serialization.Formatters;
using System;

[RequireComponent(typeof(CarController))]
public class AIP3FormationCar : MonoBehaviour
{   
    public static bool allPass = false;
    public static List<bool> isReady = new List<bool> { false, false, false, false, false };
    public static int currentGateIdx = 0;
    private CarController m_Car; // the car controller we want to use
    private MapManager m_MapManager;
    private ObstacleMapManager m_ObstacleMapManager;
    private ObstacleMap m_ObstacleMap;
    private GameObject[] m_OtherCars;
    private LineOfSightGoal m_CurrentGoal;

    public float steering;
    public float acceleration;

    private BoxCollider carCollider;
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
    private float MARGIN = 0.2f;
    private float startTime;
    private float timeLimit = 30f; // Stop searching after a certain time

    private static bool hasExecuted = false;
    private static int curCarIdx = 0;
    private int thisCarIdx;

    private List<Vector3> naivePath;
    private float targetOffset = 0.0f;

    private static List<int> carIdxOrder = new List<int> {2, 4, 0, 3, 1};
    private void Start()
    {   
        thisCarIdx = curCarIdx;
        curCarIdx++;
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        m_Car = GetComponent<CarController>();
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

        // For finding individual gates
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
        // targetPosition = gateGroup[0].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center - gateGroup[1].transform.forward * targetOffset;
        // startVel = targetPosition - startPos;
        // Debug.Log("This car is at: " + thisCarIdx);
        // (positions, times) = rrt.FindPath(startPos, targetPosition, startVel, detector, timeLimit);
        // rrt.DebugDrawPath();
        // Debug.Log($"GateGroup: {gateGroup.Count}");
        // for(int i = 0; i < gateGroup.Count - 1; i++)
        // {
        //     rrt = initRRT();
        //     startPos = gateGroup[i].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center - gateGroup[i].transform.forward * targetOffset;
        //     targetPosition = gateGroup[i+1].transform.GetChild(gateIdx).GetComponent<MeshCollider>().bounds.center - gateGroup[i + 1].transform.forward * targetOffset;
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

    private void OnDrawGizmos()
    {
        detector.DebugDrawBoundingBoxes();
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

    private List<Vector3> NaivePathPlanning(){
        // Connect the middle gate between each group
        // Make interpolation between the gates
        List<Vector3> naivePath = new List<Vector3>();
        int nMiddlePoints;
        var gateGroup = m_MapManager.GetTargetObjects();
        nMiddlePoints = 8 * (int) Mathf.Round(Vector3.Distance(transform.position, gateGroup[0].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center));
        for(int j = 0; j < nMiddlePoints; j++){
            Vector3 middlePos = Vector3.Lerp(transform.position, gateGroup[0].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center, (float)j / nMiddlePoints);
            naivePath.Add(middlePos);
        }  
        for(int i = 0; i < gateGroup.Count - 1; i++){
            Vector3 startPos = gateGroup[i].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center + gateGroup[i].transform.forward * 0f;
            Vector3 endPos = gateGroup[i+1].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center - gateGroup[i+1].transform.forward * 0f;
            // Make interpolations
            nMiddlePoints = 8 * (int) Mathf.Round(Vector3.Distance(startPos, endPos));
            for(int j = 0; j < nMiddlePoints; j++){
                Vector3 middlePos = Vector3.Lerp(startPos, endPos, (float)j / nMiddlePoints);
                naivePath.Add(middlePos);
            }
            startPos = gateGroup[i+1].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center - gateGroup[i+1].transform.forward * 0f;
            endPos = gateGroup[i+1].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center + gateGroup[i+1].transform.forward * 0f;
            nMiddlePoints = 8 * (int) Mathf.Round(Vector3.Distance(startPos, endPos));  
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
    float preSteer = 0f;
    private void NaiveControl(){


        Vector3 currentPos = transform.position;
        Vector3 targetPos;
        Vector3 targetVel;
        float lookAhead = 2f;
        float referenceSpeed = 20f;
        lookAhead = lookAhead * (referenceSpeed / GetComponent<Rigidbody>().velocity.magnitude );
        float minDistance = 4f;

        float updateIterval;
        updateIterval = 0.015f;

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
            targetPos = getRelativePos();
            // Only put the target when it is close enough or in front of the car
            if (Vector3.Dot(getRelativePos() - transform.position, transform.forward) > 0f || Vector3.Distance(getRelativePos(), transform.position) < 5f){
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
            Debug.Log($"Car {thisCarIdx} breaking, preSteer: {preSteer} ");
            if (GetComponent<Rigidbody>().velocity.magnitude < 0.05f){
                m_Car.Move(0.5f * Vector3.SignedAngle(transform.forward, m_MapManager.GetTargetObjects()[currentGateIdx+1].transform.position - m_MapManager.GetTargetObjects()[currentGateIdx].transform.position, Vector3.up), 0f, 0f, 0f);
            }
            else{
                m_Car.Move(0.5f * Vector3.SignedAngle(transform.forward, m_MapManager.GetTargetObjects()[currentGateIdx+1].transform.position - m_MapManager.GetTargetObjects()[currentGateIdx].transform.position, Vector3.up), 0f, -0.5f, 1f);
            }
            return;
        }

        // Debug.Log($"Car {thisCarIdx} is at {currentPos}, Target is at {targetPos} Distance: {Vector3.Distance(currentPos, targetPos)}");
        // Debug.Log(Vector3.Distance(currentPos, naivePath[index]) < minDistance);

        // Debug.DrawLine(targetPos, targetPos + Vector3.up * 10f, Color.green);

        targetVel = (targetPos - naivePrePos) / Time.fixedDeltaTime;
        naivePrePos = targetPos;
        Vector3 position_error = targetPos - transform.position;
        Vector3 velocity_error = targetVel - GetComponent<Rigidbody>().velocity;
        Vector3 desired_acceleration = 3f * position_error + 1f * velocity_error;

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
        steering = Mathf.Clamp(steering, -1f, 1f);
        Debug.Log($"position_error: {position_error}, velocity_error: {velocity_error}, desired_acceleration: {desired_acceleration}, steering: {steering}, acceleration: {acceleration}");
        Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        Debug.DrawLine(transform.position, transform.position + GetComponent<Rigidbody>().velocity, Color.blue);
        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);
        Debug.DrawLine(transform.position, transform.position + position_error, Color.magenta);

        // this is how you control the car
        // m_Car.Move(steering, acceleration, Math.Max(-0.5f, acceleration), 0f);
        if(Vector3.Distance(targetPos, transform.position) > 5.1f){
            acceleration = 0.1f;
        }
        m_Car.Move(steering, Math.Max(0.5f, acceleration), Math.Max(-0.5f, acceleration), -acceleration);
        preSteer = steering;
        // if(GetComponent<Rigidbody>().velocity.magnitude > 5f && thisCarIdx == 0){
        //     m_Car.Move(steering, 0f, 0f, 0f);
        // }
        // else{
        //     m_Car.Move(steering, acceleration, acceleration, 0f);
        // }
    }
    private void FixedUpdate()
    {   
        // NaiveControl();
        ControlCar();
        // if (Time.realtimeSinceStartup - startTime < 3f){
        //     m_Car.Move(0f, 1f, 1f, 0f);
        // }
        // else if (Time.realtimeSinceStartup - startTime < 10f){
        //     m_Car.Move(0f, -1f, 0f, 1f);
        // }
        // else{
        //     m_Car.Move(0f, 1f, 1f, 0f);
        // }
    }

    Vector3 rushDir;

    private void ControlCar()
    {
        Vector2 carPos = new Vector2(transform.position.x, transform.position.z);
        Vector2 carVelocity = (carPos - prevPos) / Time.fixedDeltaTime;
        float carSpeed = carVelocity.magnitude;
        prevPos = carPos;

        // Look ahead proportional to speed
        float baseLookAhead = 5f;
        float referenceSpeed = 20f;
        float lookAhead = baseLookAhead * Mathf.Max(carSpeed / referenceSpeed, 0.5f);
        lookAhead = 5f;
        // Get position and velocity that far ahead on the path
        Vector2 lookAheadPos, lookAheadVel;
        (lookAheadPos, lookAheadVel) = TrackingUtil.LookAheadPositionAndVelocity(carPos, lookAhead, positions, times);

        float futureSpeed = lookAheadVel.magnitude;
        
        // Calculate inputs
        float maxSteerAngle = 30f; // Lower = more responsive, higher = more smooth but may overshoot
        float maxAcceleration = 10f; // Same as above
        float angle = Vector2.SignedAngle(lookAheadPos-carPos, new Vector2(transform.forward.x, transform.forward.z));
        float steering = Mathf.Clamp(angle / maxSteerAngle, -1, 1);
        float acceleration = TrackingUtil.CalculateAcceleration(maxAcceleration, carPos, lookAheadPos, carSpeed, futureSpeed);

        Debug.DrawLine(Vector2ToVector3(carPos), Vector2ToVector3(carPos) + new Vector3(0, carSpeed, 0), Color.red);
        Debug.DrawLine(Vector2ToVector3(lookAheadPos), Vector2ToVector3(lookAheadPos) + new Vector3(0, futureSpeed, 0), Color.red);
        Debug.DrawLine(Vector2ToVector3(carPos), Vector2ToVector3(lookAheadPos), Color.magenta);
        
        var gateGroup = m_MapManager.GetTargetObjects();
        if (currentGateIdx >= gateGroup.Count){
            m_Car.Move(steering, acceleration, Math.Max(-0.5f, acceleration), -acceleration);
            return;
        }
        Vector3 gatePos3D = gateGroup[currentGateIdx].transform.GetChild(carIdxOrder[thisCarIdx]).GetComponent<MeshCollider>().bounds.center;

        Debug.DrawLine(Vector2ToVector3(carPos), Vector2ToVector3(carPos) + Vector3.up * 10f, Color.blue);
        Debug.DrawLine(gatePos3D, gatePos3D + Vector3.up * 10f, Color.red);

        Vector3 gatePos2D = Vector3ToVector2(gatePos3D);
        Debug.Log($"Car {thisCarIdx} is at {carPos}, Gate {currentGateIdx} is at {gatePos2D} Distance: {Vector2.Distance(carPos, gatePos2D)}");
        if (Vector2.Distance(carPos, gatePos2D) < 13f)
        {
            if (!isReady.All(b => b))
            {
                Debug.Log($"Car {thisCarIdx} velocity {GetComponent<Rigidbody>().velocity.magnitude}");
                if (GetComponent<Rigidbody>().velocity.magnitude < 0.1f)
                {
                    isReady[thisCarIdx] = true;
                    m_Car.Move(steering, 0f, 0f, 0f);
                    rushDir = gatePos3D - transform.position;
                }
                else
                {
                    Debug.Log($"This car idx: {thisCarIdx}, Ready? {isReady[thisCarIdx]}");
                    m_Car.Move(steering, 0f, -0.5f, 1f);
                }
            }
            else if (isReady.All(b => b))
            {   
                m_Car.Move(Vector3.Dot(rushDir, transform.right), Vector3.Dot(rushDir, transform.forward), 0f, 0f);
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
            NormalDrivingBehavior(steering, acceleration);
        }

        for(int i = 0; i < m_OtherCars.Length; i++)
        {
            if (i == thisCarIdx)
            {
                continue;
            }
            Vector2 orientation = Vector3ToVector2(m_OtherCars[i].transform.forward);
            Vector2 toOther = Vector3ToVector2(transform.position - m_OtherCars[i].transform.position);
            float angleToOthers = Vector2.Dot(Vector3ToVector2(GetComponent<Rigidbody>().velocity), Vector3ToVector2(m_OtherCars[i].transform.position - transform.position));
            Debug.Log($"Car {thisCarIdx} is at {carPos}, Car {i} is at {Vector3ToVector2(m_OtherCars[i].transform.position)} Angle: {angleToOthers} Distance: {Vector2.Distance(carPos, Vector3ToVector2(m_OtherCars[i].transform.position))}");
            if(angleToOthers > 0.5 && Vector2.Distance(carPos, Vector3ToVector2(m_OtherCars[i].transform.position)) < 6.5f)
            {   
                Debug.Log($"Car {thisCarIdx} is too close to car {i}");
                m_Car.Move(steering, 0f, 0f, 1f);
            }
        }



    }

    private void NormalDrivingBehavior(float steering, float acceleration)
    {
        Debug.Log($"This car idx: {thisCarIdx}, {steering}, {acceleration}");
        m_Car.Move(steering, acceleration, Math.Max(-0.5f, acceleration), -acceleration);
    }

    private RRT initRRT(){
        RRT rrt = new RRT();

        // Set the dynamic constraints
        rrt.localPlanner.acceleration = 6f;
        rrt.localPlanner.deceleration = 6f;
        rrt.localPlanner.minSpeed = 15f;
        rrt.localPlanner.maxSpeed = 25f;
        rrt.localPlanner.minTurnRadius = 1f;
        rrt.localPlanner.turnRadiusUncap = 1f;

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