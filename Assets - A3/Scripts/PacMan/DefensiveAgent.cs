using UnityEngine;
using System.Collections.Generic;
using PacMan;
using Scripts.Map;
using UnityEngine;
using System.Collections.Generic;
using PacMan;
using Scripts.Map;

namespace PacMan
{
    public class DefensiveAgent
    {
        private IPacManAgent agentManager;
        private ObstacleMap _map;
        private MazeDistanceCalculator dc;
        private bool red;

        private bool amIScared;

        private Dictionary<string, float> weights = new Dictionary<string, float>
        {
            { "enemy", 10f },
            { "food", 10f },
            { "collide_bonus", 5f }
            // etc
        };

        private Dictionary<string, int> states = new Dictionary<string, int>
        {
            { "Escape", 0 },
            { "Chase_Direct", 1 },
            { "Chase_Estimate", 2 }
            // etc
        };

        // Constructor
        public DefensiveAgent(IPacManAgent agentManager, ObstacleMap _map, MazeDistanceCalculator dc, bool red)
        {
            this.agentManager = agentManager;
            this._map = _map;
            this.dc = dc;
            this.red = red;
        }

        public PacManAction ChooseAction(
            Vector3 currPos,
            List<PacManObservation> enemies,
            List<IPacManAgent> homies)
        {
            amIScared = agentManager.IsScared();
            Vector3 center = new Vector3(0, 0, 0);

            bool enemyVisible = false;
            float closestEnemyDist = float.MaxValue;
            Vector3 closestEnemyPos = Vector3.zero;
            List<Vector3> allEnemyPositions = new List<Vector3>();
            List<Vector3> mySideEnemyPositions = new List<Vector3>();

            if (enemies?.Count > 0)
            {
                foreach (var enemy in enemies)
                {
                    allEnemyPositions.Add(enemy.Position);
                    if ((red && enemy.Position.x > 1) || (!red && enemy.Position.x < -1))
                    {
                        mySideEnemyPositions.Add(enemy.Position);
                    }

                    if (enemy.Visible)
                    {
                        enemyVisible = true;
                        float distToEnemy = Vector3.Distance(enemy.Position, currPos);
                        if (distToEnemy < closestEnemyDist)
                        {
                            closestEnemyDist = distToEnemy;
                            closestEnemyPos = enemy.Position;
                        }
                    }
                }
            }

            if (amIScared)
            {
                // If I'm scared just run away from enemies
                return Escape(currPos, center, mySideEnemyPositions, homies);
            }
            else
            {
                if (enemyVisible)
                {
                    // Debug.Log("Enemy pos: " + closestEnemyPos);
                    Debug.DrawLine(currPos, closestEnemyPos, Color.green);
                    return Chase(currPos, states["Chase_Direct"], closestEnemyPos, allEnemyPositions, homies);
                }
                else
                {
                    float mindist = float.MaxValue;
                    Vector3 pointAt = Vector3.zero;
                    foreach (var enemyPos in allEnemyPositions)
                    {
                        float dist = Vector3.Distance(currPos, enemyPos);
                        if (dist < mindist)
                        {
                            pointAt = enemyPos;
                            mindist = dist;
                        }
                    }

                    Debug.DrawLine(currPos, pointAt, Color.red);

                    return Chase(currPos, states["Chase_Estimate"], center, allEnemyPositions, homies);
                }
            }
        }

        private PacManAction Escape(Vector3 currPos,
            Vector3 targetPosition,
            List<Vector3> enemyPos,
            List<IPacManAgent> homies)
        {
            // if scared, run away from the closest ghost
            List<Vector2> allDirections = new List<Vector2>
            {
                new Vector2(-1, 0),
                new Vector2(1, 0),
                new Vector2(0, 1),
                new Vector2(0, -1)
            };

            List<Vector2> doableDirections = new List<Vector2>();

            foreach (var dir in allDirections)
            {
                Vector3 potentialNewPos = currPos;
                potentialNewPos.x += dir.x;
                potentialNewPos.z += dir.y;

                if (_map.IsGlobalPointTraversable(potentialNewPos) == ObstacleMap.Traversability.Free)
                {
                    // Don't cross the border
                    if ((red && potentialNewPos.x > 1) || (!red && potentialNewPos.x < -1))
                    {
                        doableDirections.Add(dir);
                    }
                }
            }

            float magn = 1f;
            float bestScore = float.MinValue;
            PacManAction bestAction = new PacManAction();

            foreach (var dir in doableDirections)
            {
                Vector3 potentialNewPos = currPos;
                potentialNewPos.x += magn * dir.x;
                potentialNewPos.z += magn * dir.y;

                float score = 0f;

                foreach (var enemyPosition in enemyPos)
                {
                    score += Vector3.Distance(potentialNewPos, enemyPosition);
                }

                // Collision avoidance
                foreach (var homie in homies)
                {
                    Vector3 homiePos = homie.gameObject.transform.position;
                    float dist = Vector3.Distance(potentialNewPos, homiePos);
                    // Debug.DrawLine(potentialNewPos, homiePos, Color.white);
                    if (dist < 0.6f)
                    {
                        score = float.MinValue;
                    }
                }

                if (score > bestScore)
                {
                    bestScore = score;
                    PacManAction pma = new() { AccelerationDirection = dir, AccelerationMagnitude = magn };
                    bestAction = pma;
                }

            }

            return bestAction;
        }

        private PacManAction Chase(Vector3 currPos,
            int state,
            Vector3 targetPosition,
            List<Vector3> enemyPos,
            List<IPacManAgent> homies)
        {
            List<Vector2> allDirections = new List<Vector2>
            {
                new Vector2(-1, 0),
                new Vector2(1, 0),
                new Vector2(0, 1),
                new Vector2(0, -1)
            };

            List<Vector2> doableDirections = new List<Vector2>();

            foreach (var dir in allDirections)
            {
                Vector3 potentialNewPos = currPos;
                potentialNewPos.x += dir.x;
                potentialNewPos.z += dir.y;

                if (_map.IsGlobalPointTraversable(potentialNewPos) == ObstacleMap.Traversability.Free)
                {
                    // Don't cross the border
                    if ((red && potentialNewPos.x > 1) || (!red && potentialNewPos.x < -1))
                    {
                        doableDirections.Add(dir);
                    }
                }
            }

            float magn = 1f;
            float bestScore = float.MaxValue;
            PacManAction bestAction = new PacManAction();

            foreach (var dir in doableDirections)
            {
                Vector3 potentialNewPos = currPos;
                potentialNewPos.x += magn * dir.x;
                potentialNewPos.z += magn * dir.y;

                float score = 0f;


                // Calculate the heuristics for each state
                switch (state)
                {
                    case 0:
                        // Escape is removed
                        break;
                    case 1:
                        // Chase_Direct
                        // heuristic: guide to the enemy
                        score += Vector3.Distance(potentialNewPos, targetPosition);

                        // Collision avoidance
                        foreach (var homie in homies)
                        {
                            Vector3 homiePos = homie.gameObject.transform.position;
                            // Debug.DrawLine(potentialNewPos, homiePos, Color.white);
                            score += weights["collide_bonus"] / Vector3.Distance(potentialNewPos, homiePos);
                        }

                        break;
                    case 2:
                        // Chase_Estimate
                        // heuristic: using food and all the info to guide to the enemy
                        foreach (var enemyPosition in enemyPos)
                        {
                            score += Vector3.Distance(potentialNewPos, enemyPosition);
                        }

                        // Collision avoidance
                        foreach (var homie in homies)
                        {
                            Vector3 homiePos = homie.gameObject.transform.position;
                            // Debug.DrawLine(potentialNewPos, homiePos, Color.white);
                            score += weights["collide_bonus"] / Vector3.Distance(potentialNewPos, homiePos);

                        }

                        break;
                    default:
                        break;
                }

                if (score < bestScore)
                {
                    bestScore = score;
                    PacManAction pma = new() { AccelerationDirection = dir, AccelerationMagnitude = magn };
                    bestAction = pma;
                }

            }

            // Debug.Log("PMA: " + bestAction.AccelerationDirection);

            return bestAction;
        }
    }
}