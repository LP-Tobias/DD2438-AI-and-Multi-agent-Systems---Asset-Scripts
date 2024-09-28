using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Game;
using Scripts.Map;
using System.IO;
using MAS2024.Assignment2;

namespace AstarPlanning
{
    public class Node
    {
        public Vector2 GridPosition { get; set; }
        public List<Node> Neighbors { get; set; }

        public Node(Vector2 gridPos) {
            GridPosition = gridPos;
            Neighbors = new List<Node>();
        }

        public static float Distance(Node a, Node b) {
            return Vector2.Distance(a.GridPosition, b.GridPosition);
        }

        public static Vector2 RoundVector2(Vector2 vector) {
            return new Vector2((float)Math.Round(vector.x, 3), (float)Math.Round(vector.y, 3));
        }
    }
}