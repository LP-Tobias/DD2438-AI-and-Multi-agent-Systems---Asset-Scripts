using UnityEngine;
using System.Collections.Generic;
using PacMan;
using Scripts.Map;
using Newtonsoft.Json.Linq;
using UnityEngine.UIElements;
public class UtilityAgent
{
    // Variables
    private IPacManAgent agentManager;
    private ObstacleMap _map;
    private MazeDistanceCalculator dc;
    private bool red;
    private float magn = 1f;
    private List<GameObject> scoreObjects = new List<GameObject>();
    Vector2 lastDirection;
    Vector2 beforeLastDirection;
    Vector2 beforebeforeLastDirection;
    Vector2 beforebeforebeforeLastdirection;
    Vector2 beforebeforebeforebeforeLastDirection;
    Vector2 beforebeforebeforebeforebeforeLastdirection;
    public float hysteresisFactor = 1.02f; // Bias towards the current direction
    public float randomnessFactor = 0.05f; // Randomness to add to the utility

    Vector3 lastPos; 
    public UtilityAgent(IPacManAgent agentManager, ObstacleMap _map, MazeDistanceCalculator dc, bool red)
    {
        this.agentManager = agentManager;
        this._map = _map;
        this.dc = dc;
        this.red = red;
    }

    public PacManAction ChooseAction(
        Vector3 currPos,
        Vector3 closestFood,
        Vector3 closestCapsule,
        Vector3 closestAllyCapsule,
        Vector3 closestAlly,
        Vector3 closestEnemy,
        Vector3 closestCenterCell,
        bool isPacman,
        bool isScared,
        bool isEnemyScared,
        int carriedFood,
        int scoreDifference, 
        bool redC
    )
    {
        bool noMoreCapsules = closestCapsule == Vector3.zero;
        bool noMoreFood = closestFood == Vector3.zero;
        float diagConst = .7f;

        List<Vector2> possibleDirections = new List<Vector2>()
            {
                new Vector2(-1, 0),
                new Vector2(1, 0),
                new Vector2(0, 1),
                new Vector2(0, -1),
                new Vector2(diagConst, diagConst),
                new Vector2(-diagConst, diagConst),
                new Vector2(diagConst, -diagConst),
                new Vector2(-diagConst, -diagConst),
                //new Vector2(0, 0)
            };

        List<Vector2> doableDirections = new List<Vector2>();

        foreach (var dir in possibleDirections)
        {
            Vector3 potentialNewPos = currPos;
            potentialNewPos.x += .6f * dir.x;
            potentialNewPos.z += .6f * dir.y;


            Vector3 halfPotentialLeft;
            Vector3 halfPotentialRight;
            if (dir.x != 0 && dir.y == 0)
            {
                halfPotentialLeft = currPos + 0.5f * (new Vector3(dir.x, 0, dir.y + 0.3f));
                halfPotentialRight = currPos + 0.5f * (new Vector3(dir.x, 0, dir.y - 0.3f));
            } else if(dir.x == 0 && dir.y != 0)
            {
                halfPotentialLeft = currPos + 0.5f * (new Vector3(dir.x + 0.3f, 0, dir.y));
                halfPotentialRight = currPos + 0.5f * (new Vector3(dir.x - 0.3f, 0, dir.y));
            } else if (dir.x == diagConst && dir.y == diagConst)
            {
                halfPotentialLeft = currPos + 0.5f * (new Vector3(dir.x - 0.3f, 0, dir.y+0.3f));
                halfPotentialRight = currPos + 0.5f * (new Vector3(dir.x + 0.3f, 0, dir.y-0.3f));
            }
            else if (dir.x == diagConst && dir.y == -diagConst)
            {
                halfPotentialLeft = currPos + 0.5f * (new Vector3(dir.x + 0.3f, 0, dir.y + 0.3f));
                halfPotentialRight = currPos + 0.5f * (new Vector3(dir.x - 0.3f, 0, dir.y - 0.3f));
            }
            else if (dir.x == -diagConst && dir.y == diagConst)
            {
                halfPotentialLeft = currPos + 0.5f * (new Vector3(dir.x - 0.3f, 0, dir.y - 0.3f));
                halfPotentialRight = currPos + 0.5f * (new Vector3(dir.x + 0.3f, 0, dir.y + 0.3f));
            } else
            {
                halfPotentialLeft = currPos + 0.5f * (new Vector3(dir.x + 0.3f, 0, dir.y - 0.3f));
                halfPotentialRight = currPos + 0.5f * (new Vector3(dir.x - 0.3f, 0, dir.y + 0.3f));
            }


            //Debug.DrawLine(currPos, halfPotentialLeft, Color.black);
            //Debug.DrawLine(currPos, halfPotentialRight, Color.black);
            if (_map.IsGlobalPointTraversable(potentialNewPos) == ObstacleMap.Traversability.Free
                && _map.IsGlobalPointTraversable(halfPotentialRight) == ObstacleMap.Traversability.Free
                && _map.IsGlobalPointTraversable(halfPotentialLeft) == ObstacleMap.Traversability.Free
                )
            {
                Debug.DrawLine(currPos, potentialNewPos, Color.green);
                doableDirections.Add(dir);
            }
        }

        //float dstFood = Vector3.Distance(currPos, closestFood);

        //if (dstFood < 1f)
        //{
        //    float minDist = float.MaxValue;
        //    PacManAction bestAction = new PacManAction { AccelerationDirection = doableDirections[0], AccelerationMagnitude = magn };
        //    foreach (var dir in doableDirections)
        //    {
        //        // Debug.Log(dir);
        //        Vector3 potentialNewPos = currPos;
        //        potentialNewPos.x += 0.2f * dir.x;
        //        potentialNewPos.z += 0.2f * dir.y;

        //        float dst = Vector3.Distance(potentialNewPos, closestFood);

        //        // Debug.Log("dir: " + dir + " new dist to caps:" + dst);
        //        if (dst < minDist)
        //        {
        //            minDist = dst;
        //            PacManAction pma = new() { AccelerationDirection = dir, AccelerationMagnitude = magn };
        //            bestAction = pma;
        //        }
        //    }
        //    // Debug.Log("dist to caps: " + dstFood + " dir chosen : " + bestAction.acceleration_direction);
        //    return bestAction;
        //}

        //float distCaps = Vector3.Distance(currPos, closestCapsule);

        //if (distCaps < 2f)
        //{
        //    float minDist = float.MaxValue;
        //    PacManAction bestAction = new PacManAction { AccelerationDirection = doableDirections[0], AccelerationMagnitude = magn };
        //    foreach (var dir in doableDirections)
        //    {
        //        // Debug.Log(dir);
        //        Vector3 potentialNewPos = currPos;
        //        potentialNewPos.x += 0.2f * dir.x;
        //        potentialNewPos.z += 0.2f * dir.y;

        //        float dst = Vector3.Distance(potentialNewPos, closestCapsule);

        //        //Debug.Log("dir: " + dir + " new dist to caps:" + dst);
        //        if (dst < minDist)
        //        {
        //            minDist = dst;
        //            PacManAction pma = new() { AccelerationDirection = dir, AccelerationMagnitude = magn };
        //            bestAction = pma;
        //        }
        //    }
        //    //Debug.Log("dist to caps: " + distToCapsule + " dir chosen : " + bestAction.acceleration_direction);
        //    return bestAction;
        //}

        float bestScore = float.MinValue;
        Vector2 bestDirection = doableDirections[0];

        //foreach (GameObject obj in scoreObjects)
        //{
        //    Object.Destroy(obj);
        //}
        //scoreObjects.Clear();


        foreach (var dir in doableDirections)
        {
            Vector3 potentialNewPos = currPos + 0.6f*(new Vector3(dir.x, 0, dir.y));
            float score = 0;

            float distToFood = dc.GetDistance(potentialNewPos, closestFood);
            float distToCapsule = dc.GetDistance(potentialNewPos, closestCapsule);

            if (!noMoreFood && distToFood != -1 && closestFood != new Vector3(0, 0, 0)) score += 1 / (distToFood + 1);
            if (!noMoreCapsules && distToCapsule != -1 && closestCapsule != new Vector3(0, 0, 0)) score += 1f / (distToCapsule + 1);

            // avoid cycles
            //if(Vector3.Distance(lastPos, currPos + 0.2f*(new Vector3(dir.x, 0, dir.y)))<0.15f)
            //{
            //    score -= 20;
            //}
            if(dir!=lastDirection  &&  dir == beforeLastDirection 
                && beforeLastDirection != beforebeforeLastDirection
                && beforebeforeLastDirection != beforebeforebeforeLastdirection
                && beforebeforebeforeLastdirection != beforebeforebeforebeforeLastDirection
                && beforebeforebeforebeforeLastDirection != beforebeforebeforebeforebeforeLastdirection
                && lastDirection == beforebeforeLastDirection
                && beforeLastDirection == beforebeforebeforeLastdirection
                && beforebeforeLastDirection == beforebeforebeforebeforeLastDirection
                && beforebeforebeforeLastdirection == beforebeforebeforebeforebeforeLastdirection)
            {
                score -= 3;
            }

            float distanceToAllyCapsule = Vector3.Distance(potentialNewPos, closestAllyCapsule);
            if (distanceToAllyCapsule < 0.6) // Assuming 2 units is too close
            {
                score -= 3; // Large penalty for being too close to an ally
            }

            // Stay distant from ally agents
            float distanceToAlly = Vector3.Distance(potentialNewPos, closestAlly);
            if (distanceToAlly < 0.4) // Assuming 2 units is too close
            {
                score -= 2; // Large penalty for being too close to an ally
            }

            if(closestEnemy != new Vector3(-1, -1, -1) && (currPos.x / closestEnemy.x > 0)) {
                float distanceToEnemy = dc.GetDistance(potentialNewPos, closestEnemy);
                if (isPacman)
                {
                    if (isEnemyScared)
                    {
                        score += 1 / (distanceToEnemy + 1);
                    } else
                    {
                        if(carriedFood > 0 && distanceToEnemy < 5)
                            score -= 100 / (distanceToEnemy + 1);
                        else 
                            score -= 1/(distanceToEnemy + 1);
                    }
                }
                else
                {
                    float realDist = Vector3.Distance(currPos, closestEnemy);
                    if (isScared)
                    {
                        if (realDist < 10)
                        {
                            score -= 100 / (realDist + 1);
                        }
                        else
                        {
                            score -= 100 / (distanceToEnemy + 1);
                        }
                    } else
                    {
                        if (realDist < 10)
                        {
                            score += 100 / (realDist + 1);
                        }
                        else
                        {
                            score += 100 / (distanceToEnemy + 1);
                        }
                    }
                }
            }
            
            //// If carrying max food, return to base
            if (carriedFood >= 10 || (carriedFood >= 3 && dc.GetDistance(currPos, closestFood) > 6) || (carriedFood >= 1 && Vector3.Distance(currPos, closestCenterCell)< 4))
            {
                float distanceToBase = dc.GetDistance(potentialNewPos, closestCenterCell);
                if (distanceToBase != -1)
                    score += 10 / (1 + distanceToBase); // Prefer directions that lead to base
            }

            //GameObject textObject = new GameObject("Value Text");
            //textObject.transform.position = new Vector3(potentialNewPos.x, 0.5f, potentialNewPos.z); // Adjust the y-coordinate as needed
            //textObject.transform.rotation = Quaternion.Euler(90f, 0f, 0f);

            //TextMesh textMesh = textObject.AddComponent<TextMesh>();
            //textMesh.text = score.ToString();
            //textMesh.fontSize = 100;
            //textMesh.anchor = TextAnchor.MiddleCenter;
            //textMesh.characterSize = 0.01f;

            //// Add the GameObject to the scoreObjects list
            //scoreObjects.Add(textObject);

            //if (dir == lastDirection)
            //{
            //    score *= hysteresisFactor;
            //}

            //// Apply randomness to the utility calculation
            //score += Random.Range(-randomnessFactor, randomnessFactor)/100;

            // Update best score and direction
            if (score > bestScore)
            {
                bestScore = score;
                bestDirection = dir;
            }
        }

        // Update last direction
        beforebeforebeforebeforebeforeLastdirection = beforebeforebeforebeforeLastDirection;
        beforebeforebeforebeforeLastDirection = beforebeforebeforeLastdirection;
        beforebeforebeforeLastdirection = beforebeforeLastDirection;
        beforebeforeLastDirection = beforeLastDirection;
        beforeLastDirection = lastDirection;
        lastDirection = bestDirection;

        Debug.DrawLine(currPos, currPos + new Vector3(bestDirection.x, 0, bestDirection.y), Color.cyan);
        //Debug.DrawLine(currPos, currPos + new Vector3(bestDirection.x+0.2f, 0, bestDirection.y+0.2f), Color.cyan);
        //Debug.DrawLine(currPos, currPos + new Vector3(bestDirection.x- 0.2f, 0, bestDirection.y+0.2f), Color.cyan);
        lastPos = currPos; 
        return new()
        {
            AccelerationDirection = bestDirection,
            AccelerationMagnitude = magn
        };
    }
}

//score += distToAlly * WEIGHT_ALLY; 
//if (closestEnemy.y != -1)
//{
//    if (isPacman)
//    {
//        score += distToEnemy * (isEnemyScared ? WEIGHT_ENEMY_PACMAN_CHASE : WEIGHT_ENEMY_PACMAN_SCARED);
//    }
//    else
//    {
//        score += distToEnemy * (isScared ? WEIGHT_ENEMY_GHOST_SCARED : WEIGHT_ENEMY_GHOST_CHASE);
//    }
//}
//if ( carriedFood >= 6 || (carriedFood >0 &&  isPacman && !isEnemyScared && closestEnemy.y != -1 ))
//{
//    score += distToCenter * WEIGHT_RETURN_BASE;
//}

//if (contained)
//{
//    score += WEIGHT_FOOD_COLLECTED;
//}
