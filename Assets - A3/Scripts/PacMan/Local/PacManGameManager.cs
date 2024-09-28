using System;
using System.Collections.Generic;
using System.Linq;
using Scripts.Map;
using Scripts.Utils;
using UnityEngine;
using UnityEngine.Serialization;

namespace PacMan
{
    public class PacManGameManager : MonoBehaviour
    {
        public MapManager mapManager;
        public ObstacleMapManager obstacleMap;
        public GameObject agentPrefab;
        public GameObject foodPrefab;
        public GameObject capsulePrefab;

        public int blueFood;
        public int redFood;
        public int redScore;
        public int blueScore;

        protected float StartTime;
        public float matchTime;
        public float matchLength = 240; //From original 3000 / 4

        public bool finished = false;
        public int agentsPerTeam;
        public string teamName = "";

        public List<GameObject> agents;
        public List<GameObject> redAgents;
        public List<GameObject> blueAgents;
        public List<GameObject> foodList;
        public List<GameObject> capsules;
        public bool started;

        private IPacManWorkerInterface _pacManWorker;
        public bool AutomaticRestart;
        private Grid _grid;


        void Awake()
        {
            StartTime = Time.time;
            _grid = GameManagerUtils.CopyComponent(mapManager.grid, gameObject);
            obstacleMap = transform.Find("Map/Grid").GetComponent<ObstacleMapManager>();
            obstacleMap.mapManager.SyncObjectsToPositions();
            _grid.cellSize = Vector3.one;
            _grid.cellSwizzle = GridLayout.CellSwizzle.XZY;
            _pacManWorker = GetComponent<IPacManWorkerInterface>();
        }

        public virtual void Start()
        {
            StartGame();
            agents.ForEach(agent => agent.transform.parent = gameObject.transform);
        }

        public void RestartGame()
        {
            foodList.ForEach(food => _pacManWorker.RemoveObject(food));
            capsules.ForEach(capsule => _pacManWorker.RemoveObject(capsule));
            StartGame();
        }

        public virtual void StartGame()
        {
            StartTime = Time.time;

            if (agents.Count == mapManager.startPositions.Count) //TODO: Variable agent counts map to map?
            {
                agents.ForEach(agent => _pacManWorker.ResetAgent(agent));
            }

            agentsPerTeam = mapManager.startPositions.Count / 2;
            if (agents == null || agents.Count == 0)
            {
                agents = new List<GameObject>();
                AddTeam("Blue");
                AddTeam("Red");
            }


            CreateEdibles();
            started = true;
            finished = false;
        }

        public void AddTeam(string team_tag)
        {
            int index = team_tag == "Blue" ? 1 : 2;
            for (int i = 0; i < agentsPerTeam; i++)
            {
                var position = mapManager.startPositions[i + (index - 1) * agentsPerTeam];
                var agent = _pacManWorker.CreateAgent(agentPrefab, gameObject, mapManager.transform.Find("Grid/Starts").transform.position + position, team_tag);

                agents.Add(agent);
                if (team_tag == "Blue")
                {
                    blueAgents.Add(agent);
                }
                else
                {
                    redAgents.Add(agent);
                }
            }
        }

        private void CreateEdibles()
        {
            foodList = mapManager.targetPositions.Select(pos => _pacManWorker.CreateEdible(gameObject, foodPrefab, mapManager.transform.Find("Grid/Targets").position + pos)).ToList();
            var capsulesObject = mapManager.transform.Find("Grid/Capsules");
            capsules = new List<GameObject>();
            foreach (Transform transform in capsulesObject)
            {
                capsules.Add(_pacManWorker.CreateEdible(gameObject, capsulePrefab, capsulesObject.position + transform.localPosition));
            }
        }

        // Update is called once per frame
        public void FixedUpdate()
        {
            if (started)
            {
                if (!finished)
                {
                    matchTime = Time.time - StartTime;
                    blueFood = foodList.FindAll(food => TeamAssignmentUtil.CheckTeam(food) == Team.Blue).Count;
                    redFood = foodList.FindAll(food => TeamAssignmentUtil.CheckTeam(food) == Team.Red).Count;
                    blueScore = blueFood - redFood;
                    redScore = redFood - blueFood;
                    if (matchTime > matchLength) finished = true;
                }
            }

            if (finished && AutomaticRestart)
            {
                RestartGame();
            }
        }

        public void DropFood(IPacManAgentManager pacManAgentAgent, bool success)
        {
            if (pacManAgentAgent.GetCarriedFoodCount() > 0)
            {
                var worldToCell = _grid.LocalToCell(pacManAgentAgent.gameObject.transform.localPosition);
                var agentPos = new Vector2Int(worldToCell.x, worldToCell.y);

                Func<float, bool> predicate = null;
                if (TeamAssignmentUtil.CheckTeam(pacManAgentAgent.gameObject) == Team.Red && success)
                    predicate = x => x >= 0;
                if (TeamAssignmentUtil.CheckTeam(pacManAgentAgent.gameObject) == Team.Blue && success)
                    predicate = x => x < 0;
                if (TeamAssignmentUtil.CheckTeam(pacManAgentAgent.gameObject) == Team.Red && !success)
                    predicate = x => x < 0;
                if (TeamAssignmentUtil.CheckTeam(pacManAgentAgent.gameObject) == Team.Blue && !success)
                    predicate = x => x >= 0;

                var tempObst = new List<GameObject>();
                tempObst.AddRange(foodList);
                tempObst.AddRange(capsules);
                var map = RebuildObstacleMap(tempObst);

                var nearestFreeCells = map.traversabilityPerCell.ToList()
                    .FindAll(pair => pair.Value == ObstacleMap.Traversability.Free && predicate.Invoke(pair.Key.x)) //TODO: Does not filter food or capsules in the way
                    .Select(pair => pair.Key)
                    .Select(cell => (cell, cell - agentPos))
                    .OrderBy(tuple => Mathf.Abs(tuple.Item2.x) + Mathf.Abs(tuple.Item2.y))
                    .Select(tuple => tuple.cell)
                    .Take(pacManAgentAgent.GetCarriedFoodCount())
                    .Select(vec => _grid.CellToWorld(new Vector3Int(vec.x, vec.y, 0)) + new Vector3(_grid.cellSize.x / 2, 0.4f, _grid.cellSize.y / 2))
                    .ToList();

                foodList.AddRange(nearestFreeCells
                    .Select(cell => _pacManWorker.CreateEdible(gameObject, foodPrefab, cell)).ToList()
                );

                pacManAgentAgent.SetFoodCount(0);
            }
        }

        public ObstacleMap RebuildObstacleMap(List<GameObject> additionalObjects)
        {
            var obstacleObjects = mapManager.GetObstacleObjects();
            mapManager.SyncObjectsToPositions();
            obstacleObjects.AddRange(additionalObjects);
            var ObstacleMap = new ObstacleMap(obstacleObjects, _grid);
            ObstacleMap.blockedUnfilledMargin = 0.9f;
            ObstacleMap.partialUnfilledMargin = 0.2f;
            ObstacleMap.GenerateMap();
            return ObstacleMap;
        }

        public void EatFood(IPacManAgentManager pacManAgentManager, GameObject gameObject)
        {
            if (foodList.Contains(gameObject))
            {
                foodList.Remove(gameObject);
                _pacManWorker.RemoveObject(gameObject);

                pacManAgentManager.SetFoodCount(pacManAgentManager.GetCarriedFoodCount() + 1);
            }
        }

        public void EatCapsule(IPacManAgentManager pacManAgentAgent, GameObject gameObject)
        {
            if (capsules.Contains(gameObject))
            {
                var scaredTeam = pacManAgentAgent.CompareTag("Red") ? blueAgents : redAgents;

                scaredTeam.ForEach(agent =>
                {
                    var otherAgent = agent.GetComponent<IPacManAgentManager>();
                    otherAgent.SetScared(true, Time.fixedTime + 10);
                });

                capsules.Remove(gameObject);
                _pacManWorker.RemoveObject(gameObject);
            }
        }
    }
}