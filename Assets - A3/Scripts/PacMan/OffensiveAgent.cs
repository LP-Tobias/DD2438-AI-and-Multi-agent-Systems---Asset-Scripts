using UnityEngine;
using System.Collections.Generic;
using PacMan;
using Scripts.Map;

public class OffensiveAgent
{
    // Variables
    private IPacManAgent agentManager;
    private ObstacleMap _map;
    private MazeDistanceCalculator dc;
    private bool red; 
    // Initialize
    public OffensiveAgent(IPacManAgent agentManager, ObstacleMap _map , MazeDistanceCalculator dc, bool red)
    {
        this.agentManager = agentManager;
        this._map = _map;
        this.dc = dc;
        this.red = red; 
    }

    public PacManAction ChooseAction(
        Vector3 currPos,
        Vector3 foodPosition, 
        Vector3 capsulePosition, 
        int carriedFood, 
        List<PacManObservation> enemies, 
        int foodLeft,
        int returnedFood
    )
    {
        // general variables:
        List<Vector2> possibleDirections = new List<Vector2>()
        {
            new Vector2(-1, 0),
            new Vector2(1, 0),
            new Vector2(0, 1),
            new Vector2(0, -1),
            //new Vector2(-.3f, -.3f),
            //new Vector2(.3f, -.3f),
            //new Vector2(.3f, .3f),
            //new Vector2(-.3f, .3f)
            // new Vector2(0, 0)
            // TODO: diagonal directions don't really work rn
            // TODO: debug pacman sometimes hit a wall, maybe because the potential pos is showed as free while it is occupied, need to change how we calculate potential new pos? 
        };

        List<Vector2> doableDirections = new List<Vector2>();

        //Vector3 currPos = agentManager.transform.position; 

        foreach(var dir in possibleDirections)
        {
            Vector3 potentialNewPos = currPos;
            potentialNewPos.x += 0.7f*dir.x;
            potentialNewPos.z += 0.7f*dir.y;
            //Debug.Log("curr pos :" + currPos + " potential new pos " + potentialNewPos + " dir : " + dir);
            if(_map.IsGlobalPointTraversable(potentialNewPos) == ObstacleMap.Traversability.Free)
            {
                //Debug.Log("adding the dir to doable dirs");
                doableDirections.Add(dir);
            }
        }

        float magn = 1f;

        bool noMoreCapsules = capsulePosition == Vector3.zero;
        bool noMoreFood = foodPosition == Vector3.zero;

        PacManAction bestAction = new PacManAction { AccelerationDirection = doableDirections[0], AccelerationMagnitude = magn };

        float distToCapsule = Vector3.Distance(currPos, capsulePosition);

        if (distToCapsule < 2f)
        {
            float minDist = float.MaxValue;
            foreach (var dir in doableDirections)
            {
                // Debug.Log(dir);
                Vector3 potentialNewPos = currPos;
                potentialNewPos.x += 0.2f*dir.x;
                potentialNewPos.z += 0.2f*dir.y;

                float dst = Vector3.Distance(potentialNewPos, capsulePosition);

                //Debug.Log("dir: " + dir + " new dist to caps:" + dst);
                if (dst < minDist)
                {
                    minDist = dst;
                    PacManAction pma = new() { AccelerationDirection = dir, AccelerationMagnitude = magn };
                    bestAction = pma;
                }
            }
            //Debug.Log("dist to caps: " + distToCapsule + " dir chosen : " + bestAction.acceleration_direction);
            return bestAction;
        }

        float dstFood = Vector3.Distance(currPos, foodPosition);

        if (dstFood < 1f)
        {
            float minDist = float.MaxValue;
            foreach (var dir in doableDirections)
            {
                // Debug.Log(dir);
                Vector3 potentialNewPos = currPos;
                potentialNewPos.x += 0.2f * dir.x;
                potentialNewPos.z += 0.2f * dir.y;

                float dst = Vector3.Distance(potentialNewPos, foodPosition);

                // Debug.Log("dir: " + dir + " new dist to caps:" + dst);
                if (dst < minDist)
                {
                    minDist = dst;
                    PacManAction pma = new() { AccelerationDirection = dir, AccelerationMagnitude = magn };
                    bestAction = pma;
                }
            }
            // Debug.Log("dist to caps: " + dstFood + " dir chosen : " + bestAction.acceleration_direction);
            return bestAction;
        }

        float bestScore = float.MinValue;

        foreach (var dir in doableDirections)
        {
            // Debug.Log(dir);
            Vector3 potentialNewPos = currPos;
            potentialNewPos.x += dir.x;
            potentialNewPos.z += dir.y;

            //potentialNewPos.x = RoundToNearestHalf(potentialNewPos.x);
            //potentialNewPos.z = RoundToNearestHalf(potentialNewPos.z);
            Vector3 actualNewPos = currPos;
            actualNewPos.x += agentManager.GetVelocity().magnitude * dir.x;
            actualNewPos.z += agentManager.GetVelocity().magnitude * dir.y;

            bool contained = false;
            if (Vector3.Distance(actualNewPos, foodPosition) < 0.5f) 
            {
                // Debug.Log("containeeddir : " + dir + " actualNewPos: " + actualNewPos);
                contained = true;
                // stg to be done to foodscore
            }

            bool returned = false;
            if (carriedFood > 0 && actualNewPos.x >= -0.1f)
            {
                // Debug.Log("returned to base: "+ returnedFood);
                returned = true;
            }
            float distToFood = dc.GetDistance(potentialNewPos, foodPosition);
            if (distToFood == -1) distToFood = float.MaxValue;
            //if (distToFood < minDist)
            //{
            //    minDist = distToFood;
            //    PacManAction pma = new PacManAction { acceleration_direction = dir, acceleration_magnitude = magn };
            //    bestAction = pma;
            //}

            float distToGhost = float.MaxValue; // not scared ghosts
            float distToScared = float.MaxValue; // scared ghosts

            if (enemies?.Count > 0)
            {
                foreach (var enemy in enemies)
                {
                    if (enemy.Position == Vector3.zero || !enemy.Visible) continue;
                    if (enemy.IsGhost)
                    {
                        distToGhost = Mathf.Min(distToGhost, Vector3.Distance(enemies[0].Position, potentialNewPos));
                    }
                    //if (enemy.isScared)
                    // Debug.Log(enemy.position + " :" + enemy.readingDispersion); //Uniform random
                }
            }

            bool inEnemySide = red? potentialNewPos.x < 0 : potentialNewPos.x > 0;

            float dstToCaps = dc.GetDistance(potentialNewPos, capsulePosition);
            if (dstToCaps == -1) dstToCaps = float.MaxValue;

            float weightedScoreDiff = -(contained?-1:0+ foodLeft) * 50;
            float weightedInEnemySide = (inEnemySide?1:0) * 10;
            float weightedDistToFood = distToFood * -1;
            float weightedDistToCapsule = dstToCaps * -0.5f;
            float weightedDistToGhost = distToGhost * 5;
            float weightedCarriedFood = (contained?1:0+ carriedFood) * -2;
            float weightedReturnedFood = (returned ? agentManager.GetCarriedFoodCount() : 0 + returnedFood) * 10;

            float score = weightedScoreDiff+ weightedInEnemySide + weightedCarriedFood + weightedReturnedFood;

            if (!noMoreFood)
            {
                score += weightedDistToFood;
            }
            if (!noMoreCapsules)
            {
                score += weightedDistToCapsule;
            }
            if(distToGhost != float.MaxValue)
            {
                score += weightedDistToGhost;
            }

            if (score > bestScore)
            {
                bestScore = score;
                PacManAction pma = new PacManAction { AccelerationDirection = dir, AccelerationMagnitude = magn };
                bestAction = pma;
            }

            //Debug.Log("dir :" + dir + " score: " + score + " weightedScoreDiff:" + weightedScoreDiff + 
            //    " // weighted carried food: " + weightedCarriedFood +
            //    " // weightedInEnemySide: " + weightedInEnemySide + 
            //    " // weighted Dist To Food : " + weightedDistToFood + 
            //    " // weighted dist to capsule : " + weightedDistToCapsule + 
            //    " // weighted dist to ghost: "+ weightedDistToGhost);

            // Debug.Log("dir: "+ dir + "// score: " + score + " // food_score: " + weightedDistToFood + " // caps_score: " + weightedDistToCapsule+" //enemyside: "+weightedInEnemySide);
        }

        // Debug.Log("chosen direction : " + bestAction.acceleration_direction);
        return bestAction;
    }
}