using System.Collections.Generic;
using System;
using System.Linq;
using UnityEngine;
using ORCA;

namespace AVO
{
    public class Accel_VO
    {
        private const int maxSteps = 100;
        float AVO_EPSILON = 0.00001f;
        private GameObject[] Others;
        private List<GameObject> Obstacles;
        private Vector2 m_position;
        private Vector2 m_velocity;
        private Vector2 m_prefVelocity;
        
        private float radius;
        private float tau; // timeHorizon
        private float delta; // I think this is the acceleration interval
        private float timeStep;
        // I might need acceleration Interval, timeStep (delta time?), timeHorizon
        
        
        public Vector2 newVelocity;
        
        private float maxSpeed;
        private float maxAccel;
        private IList<Line> orcaLines = new List<Line>();
        private LinkedList<Vector2> boundary = new LinkedList<Vector2>();
        
        // the agent has some more params, write while check...
        
        // Constructor
        public Accel_VO(GameObject[] others, 
            List<GameObject> obstacles, 
            Vector2 position, 
            Vector2 velocity, 
            Vector2 prefVelocity, 
            float radius, float tau, float delta, float timeStep, float maxSpeed, float maxAccel)
        {
            Others = others;
            Obstacles = obstacles;
            m_position = position;
            m_velocity = velocity;
            m_prefVelocity = prefVelocity;
            this.radius = radius;
            this.tau = tau;
            this.delta = delta;
            this.timeStep = timeStep;
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
        }
        
        // Utility functions
        public Vector2 tangentPoint(Vector2 p, float r)
        {
            float pLengthSq = RVOMath.absSq(p);
            float l = RVOMath.sqrt(pLengthSq - r * r);
            return new Vector2(l * p.x - r * p.y, r * p.x + l * p.y) * (l / pLengthSq);
        }

        public Vector2 boundaryAVO(Vector2 p, Vector2 v, float r, float delta, float t)
        {
            float e = Mathf.Exp(-t / delta) - 1.0f;
            float temp = 1.0f / (t + delta * e);
            
            Vector2 dc = (-e * p - (delta * e + (e + 1.0f) * t) * v) * temp * temp;
            Vector2 rdrdc = (delta + t / e) * dc;
            
            Vector2 c = (delta * e * v - p) * temp;
            float radius = r * temp;
            
            return c - rdrdc + tangentPoint(rdrdc, radius) - v;
        }

        public Vector2 centerAVO(Vector2 p, Vector2 v, float delta, float t)
        {
            float temp = delta * (Mathf.Exp(-t / delta) - 1.0f);
            return (temp * v - p) / (t + temp) - v;
        }

        public (Vector2, Vector2) circleCircleIntersection(float R, float r, Vector2 p)
        {
            float RSq = R * R;
            float rSq = r * r;
            float pxSq = p.x * p.x;
            float pySq = p.y * p.y;
            float squareRoot = RVOMath.sqrt(-pxSq * (pxSq * pxSq + pySq * pySq + 2.0f * pxSq * (pySq - rSq - RSq) + (rSq - RSq) * (rSq - RSq) - 2.0f * pySq * (rSq + RSq)));
            
            float xNum = pxSq * pxSq + pxSq * pySq - pxSq * rSq + pxSq * RSq;
            float yNum = pxSq * p.y + p.y * pySq - p.y * rSq + p.y * RSq;
            float yDenom = 2.0F * (pxSq + pySq);
            float xDenom = yDenom * p.x;
            return (new Vector2((xNum - p.y * squareRoot) / xDenom, (yNum + squareRoot) / yDenom),
                    new Vector2((xNum + p.y * squareRoot) / xDenom, (yNum - squareRoot) / yDenom));
        }
        
        public float distSqPointLineSegment(Vector2 vector1, Vector2 vector2, Vector2 vector3) {
            float r = Vector2.Dot((vector3 - vector1), (vector2 - vector1)) / RVOMath.absSq(vector2 - vector1);

            if (r < 0.0F) 
            {
                return RVOMath.absSq(vector3 - vector1);
            }
            if (r > 1.0F) 
            {
                return RVOMath.absSq(vector3 - vector2);
            }
            return RVOMath.absSq(vector3 - (vector1 + r * (vector2 - vector1)));
        }
        
        public float leftOf(Vector2 vector1, Vector2 vector2, Vector2 vector3) {
            return RVOMath.det(vector1 - vector3, vector2 - vector1);
        }

        public float radiusAVO(float r, float delta, float t) 
        {
            return r / (t + delta * (Mathf.Exp(-t / delta) - 1.0f));
        }
        
        // Compute new velo
        public void AVO_compute()
        {
            boundary = new LinkedList<Vector2>();
            orcaLines = new List<Line>();
            
            float speed = RVOMath.abs(m_velocity);
            Vector2 direction = m_velocity / speed;
            
            Line speedLine = new Line();
            // Insert maximum speed line.
            speedLine.point = (maxSpeed - speed) * direction;
            speedLine.direction = new Vector2(-direction.y, direction.x);
            orcaLines.Add(speedLine);
            // Insert minimum speed line.
            speedLine.point = (maxSpeed - speed) * -direction;
            speedLine.direction = new Vector2(direction.y, -direction.x);
            orcaLines.Add(speedLine);
            
            int numObstLines = orcaLines.Count();
            
            Line line = new Line();
            // Create Agent ORCA lines
            
            // my position and velocity
            Vector2 currentPos = m_position;
            Vector2 currentVelocity = m_velocity;
            float combinedRadius = 2 * radius;
            float combinedMaxAccel = 2 * maxAccel;
            
            foreach (var Other in Others)
            {
                Vector2 otherPos = new Vector2(Other.transform.position.x, Other.transform.position.z);
                Vector2 otherVelocity = new Vector2(Other.GetComponent<Rigidbody>().velocity.x, Other.GetComponent<Rigidbody>().velocity.z);
                
                Vector2 relativePosition = -(currentPos - otherPos);
                Vector2 relativeVelocity = -(currentVelocity - otherVelocity);
                float relativePositionSq = RVOMath.absSq(relativePosition);
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);
                
                LinkedList<Vector2> boundary = new LinkedList<Vector2>();
                bool convex = false;
                float left = 0f;
                Vector2 cutoffCenter;
                float cutoffRadius = 0f;
                
                float determinant = RVOMath.det(relativePosition, relativeVelocity);
                if (determinant > 0f)
                {
                    left = -1.0f;
                }
                else
                {
                    left = 1.0f;
                }

                if (relativePositionSq <= combinedRadiusSq)
                {
                    //collision
                    Vector2 centerDt = centerAVO(relativePosition, relativeVelocity, delta, timeStep);
                    float radiusDt = radiusAVO(combinedRadius, delta, timeStep);
                    Vector2 centerTau = centerAVO(relativePosition, relativeVelocity, delta, tau);
                    float radiusTau = radiusAVO(combinedRadius, delta, tau);
                    
                    Vector2 centerVec = centerDt - centerTau;
                    float centerVecSq = RVOMath.absSq(centerVec);
                    float diffRadius = left * (radiusDt - radiusTau);
                    float diffRadiusSq = diffRadius * diffRadius;

                    if (centerVecSq > diffRadiusSq)
                    {
                        float l = RVOMath.sqrt(centerVecSq - diffRadiusSq);
                        Vector2 unitDir = new Vector2(l * centerVec.x - diffRadius * centerVec.y, diffRadius * centerVec.x + l * centerVec.y) / centerVecSq;
                        boundary.AddLast(centerTau + radiusTau * new Vector2(-left * unitDir.y, left * unitDir.x));
                        boundary.AddLast(centerDt + radiusDt * new Vector2(-left * unitDir.y, left * unitDir.x));
                        
                        cutoffCenter = centerTau;
                        cutoffRadius = radiusTau;
                    }
                    else 
                    {
                        cutoffCenter = centerDt;
                        cutoffRadius = radiusDt;
                    }
                }
                else
                {
                    // no-collision
                    float discr = combinedRadiusSq * RVOMath.absSq(relativeVelocity) - determinant * determinant;

                    if (discr >= 0f)
                    {
                        convex = (RVOMath.sqrt(discr) - Vector2.Dot(relativePosition, relativeVelocity)) >= 0f;
                    }
                    else
                    {
                        convex = false;
                    }

                    float lastTime = timeStep;
                    for (int i = 0; i <= maxSteps; i++)
                    {
                        float t = timeStep + (float) i * (tau - timeStep) / (float) maxSteps;
                        boundary.AddFirst(boundaryAVO(relativePosition, relativeVelocity, left * combinedRadius, delta, t));
                        float boundaryFrontX = boundary.First.Value.x;
                        if (boundaryFrontX != boundaryFrontX) {
                            // Boundary not well-defined.
                            boundary.RemoveFirst();
                            convex = true;
                            break;
                        }
                        
                        if (convex && i >= 2 && left * leftOf(GetElementAt(boundary, 0), GetElementAt(boundary, 1), GetElementAt(boundary, 2)) > AVO_EPSILON) {
                            // Failed to detect undefined part of boundary, and now running into
                            // nonconvex vertex.
                            boundary.RemoveFirst();
                            boundary.RemoveFirst();
                            break;
                        }

                        lastTime = t;
                    }
                    
                    cutoffCenter = centerAVO(relativePosition, relativeVelocity, delta, lastTime);
                    cutoffRadius = radiusAVO(combinedRadius, delta, lastTime);
                    
                    //DrawCircle(Vector2ToVector3(currentPos + cutoffCenter), cutoffRadius, Color.red);
                    
                    if(!convex)
                    {
                        List<Vector2> intersections = new List<Vector2>();
                        float rSq = delta * delta * combinedMaxAccel * combinedMaxAccel;

                        for (int i = 0; i < boundary.Count - 1; i++)
                        {
                            Vector2 p = GetElementAt(boundary, i);
                            Vector2 v = GetElementAt(boundary, i + 1) - p;
                            float vSq = RVOMath.absSq(v);
                            float detPv = RVOMath.det(p, v);
                            float discr2 = rSq * vSq - detPv * detPv;
                            
                            float distSq1 = RVOMath.absSq(p);
                            float distSq2 = RVOMath.absSq(p + v);
                            
                            if (discr2 > 0.0f && ((distSq1 < rSq && distSq2 > rSq) ||
                                                  (distSq1 >= rSq && distSq2 <= rSq) ||
                                                  (distSq1 >= rSq && distSq2 > rSq &&
                                                   RVOMath.absSq(p - (p * v / vSq) * v) < rSq))) {
                                float discrSqrt = RVOMath.sqrt(discr2);
                                float t1 = -(Vector2.Dot(p, v) + discrSqrt) / vSq;
                                float t2 = -(Vector2.Dot(p, v) - discrSqrt) / vSq;

                                if (t1 >= 0.0f && t1 < 1.0f) {
                                    // Segment intersects disc.
                                    intersections.Add(p + t1 * v);
                                }

                                if (t2 >= 0.0f && t2 < 1.0f) {
                                    // Segment intersects disc.
                                    intersections.Add(p + t2 * v);
                                }
                            }
                        }
                        
                        float velocityPlusCutoffRadius = delta * combinedMaxAccel + cutoffRadius;
                        bool cutoffInDisc = RVOMath.absSq(cutoffCenter) < velocityPlusCutoffRadius * velocityPlusCutoffRadius &&
                                            (RVOMath.absSq(boundary.First.Value) < rSq || 
                                            left * leftOf(new Vector2(0f, 0f), cutoffCenter, boundary.First.Value) > 0.0f);
                        
                        if (!cutoffInDisc) {
                            if (intersections.Count != 0) {
                                // Line between first and last left intersection.
                                line.direction = left * (intersections[intersections.Count - 1] - intersections[0]).normalized;
                                line.point = 0.5f * intersections[0];
                                orcaLines.Add(line);
                                continue;
                            }
                            // Nothing in disc.
                            continue;
                        }
                        // Cutoff arc intersects disc. Connect last intersection to disc and
                        // replace boundary.
                        if (intersections.Count == 0 && RVOMath.absSq(GetElementAt(boundary, 0)) <= rSq) {
                            // Boundary too short to intersect disc.
                            intersections.Add(GetElementAt(boundary, boundary.Count - 1));
                        }
                        else if (intersections.Count != 0 && RVOMath.absSq(GetElementAt(boundary, 0) - intersections[intersections.Count - 1]) < AVO_EPSILON) {
                            // Boundary starts exactly at disc.
                            intersections.Clear();
                        }
                        
                        if (intersections.Count != 0) {
                            Vector2 q = tangentPoint(cutoffCenter - intersections[intersections.Count - 1], -left * cutoffRadius) + intersections[intersections.Count - 1];

                            if (RVOMath.absSq(q) > rSq) {
                                // Compute intersection point of cutoff arc and disc and create line.
                                (Vector2 p_first, Vector2 p_second) = circleCircleIntersection(delta * combinedMaxAccel, cutoffRadius, cutoffCenter);

                                if (-left * leftOf(intersections[intersections.Count - 1], cutoffCenter, p_first) > 0.0f) {
                                    q = p_first;
                                } else {
                                    q = p_second;
                                }

                                line.direction = left * (intersections[intersections.Count - 1] - q).normalized;
                                line.point = 0.5f * q;
                                orcaLines.Add(line);
                                continue;
                            }
                            boundary.Clear();
                            boundary.AddLast(q);
                            boundary.AddLast(intersections[intersections.Count - 1]);
                        } else {
                            boundary.Clear();
                        }
                    }
                }
                
                // DrawCircle(Vector2ToVector3(currentPos + cutoffCenter), cutoffRadius, Color.red);
                // DrawBoundary(boundary, Color.cyan);
                
                // Treat as convex obstacle.
                if (boundary.Count() <= 1 || left * leftOf(cutoffCenter, GetElementAt(boundary, 0), new Vector2(0f, 0f)) >= 0.0f) {
                    // Project on cutoff center.
                    float wLength = RVOMath.abs(cutoffCenter);
                    Vector2 unitW = cutoffCenter / -wLength;

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    line.point = 0.5f * (cutoffRadius - wLength) * unitW;
                    orcaLines.Add(line);
                    continue;
                }
                // Find closest point on boundary.
                float minDistSq = float.MaxValue;
                int minSegment = 0;

                for (int i = 0; i < boundary.Count - 1; ++i) { 
                    float distSq = distSqPointLineSegment(GetElementAt(boundary, i), GetElementAt(boundary, i + 1), new Vector2(0f, 0f));
                    if (distSq < minDistSq) {
                        minDistSq = distSq;
                        minSegment = i;
                    }
                }

                line.direction = (float) left * (GetElementAt(boundary, minSegment + 1) - GetElementAt(boundary, minSegment).normalized);
                line.point = 0.5f * GetElementAt(boundary, minSegment);
                orcaLines.Add(line);
            }
            
            int lineFail = linearProgram_shell(orcaLines, maxAccel * delta, m_prefVelocity - m_velocity, false, ref newVelocity);
            
            if (lineFail == 0)
            {
                linearProgram3(orcaLines, numObstLines, 0, maxAccel * delta, ref newVelocity);
            }
            
            this.newVelocity += m_velocity;
            
        }
        
        Vector2 GetElementAt(LinkedList<Vector2> list, int index)
        {
            if (index < 0 || index >= list.Count)
            {
                throw new ArgumentOutOfRangeException(nameof(index));
            }

            var current = list.First;
            for (int i = 0; i < index; i++)
            {
                current = current.Next;
            }
    
            return current.Value;
        }
        
        private int linearProgram_shell(IList<Line> lines, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }
        
        private bool linearProgram1(IList<Line> lines, int lineNo, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            float dotProduct = Vector2.Dot(lines[lineNo].point, lines[lineNo].direction);
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (Vector2.Dot(optVelocity , lines[lineNo].direction) > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = Vector2.Dot(lines[lineNo].direction, (optVelocity - lines[lineNo].point));
            
                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }
        
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (Vector2.Dot(lines[i].direction, lines[j].direction) > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (linearProgram_shell(projLines, radius, new Vector2(-lines[i].direction.y, lines[i].direction.x), true, ref result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
        
        public void DrawCircle(Vector3 center, float radius, Color color)
        {
            int segments = 360;
            float angleStep = 360f / segments;
            Quaternion rotation = Quaternion.Euler(0f, angleStep, 0);

            Vector3 startVertex = center + Vector3.right * radius;
            Vector3 nextVertex = startVertex;

            for (int i = 0; i <= segments; i++)
            {
                Vector3 endVertex = center + rotation * (nextVertex - center);
                Debug.DrawLine(nextVertex, endVertex, color);
                nextVertex = endVertex;
            }
        }
        
        private Vector3 Vector2ToVector3(Vector2 v)
        {
            return new Vector3(v.x, 0.4f, v.y);
        }
        
        public void DrawBoundary(LinkedList<Vector2> boundary, Color color)
        {
            if (boundary.Count < 2) return; // Not enough points to draw a line

            LinkedListNode<Vector2> currentNode = boundary.First;
            Vector2 startPoint = currentNode.Value;
            Vector2 previousPoint = startPoint;

            while (currentNode.Next != null)
            {
                currentNode = currentNode.Next;
                Vector2 currentPoint = currentNode.Value;

                Debug.DrawLine(new Vector3(previousPoint.x, 0, previousPoint.y), new Vector3(currentPoint.x, 0, currentPoint.y), color);

                previousPoint = currentPoint;
            }

            // Draw a line from the last point to the first point to close the boundary
            Debug.DrawLine(new Vector3(previousPoint.x, 0, previousPoint.y), new Vector3(startPoint.x, 0, startPoint.y), color);
        }
    }
}