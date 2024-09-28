using System.Collections.Generic;
using System;
using UnityEngine;

namespace ORCA
{
    public class VelocityObstacle
    {
        private GameObject[] Others;
        private List<GameObject> Obstacles;
        private Vector2 m_position;
        private Vector2 m_velocity;
        private Vector2 m_prefVelocity;
        
        private float radius;
        private float tau;
        private float tau_obj;
        
        public Vector2 newVelocity;
        
        private float maxSpeed;
        private IList<Line> orcaLines = new List<Line>();
        
        // Constructor
        public VelocityObstacle(GameObject[] others, List<GameObject> obstacles, Vector2 position, Vector2 velocity, Vector2 prefVelocity, float maxSpeed ,float radius, float tau, float tauObj)
        {
            this.Others = others;
            this.Obstacles = obstacles;
            this.m_position = position;
            this.m_velocity = velocity;
            this.m_prefVelocity = prefVelocity;
            this.maxSpeed = maxSpeed;
            this.radius = radius;
            this.tau = tau;
            this.tau_obj = tauObj;
        }
        
        public void VO_Compute(bool ComputeStatic, bool Visualize_Static, bool Visualize_Dynamic, float ValidDist)
        {
            orcaLines = new List<Line>();
            // Sometimes compute static is tricky...
            if (ComputeStatic){VO_Static(Visualize_Static, orcaLines, ValidDist);}
            
            VO_Dynamic(Visualize_Dynamic, orcaLines, ValidDist);
            
            int lineFail = linearProgram_shell(orcaLines, maxSpeed, m_prefVelocity, false, ref newVelocity);
            
            if (lineFail < orcaLines.Count)
            {
                linearProgram3(orcaLines, 0, lineFail, maxSpeed, ref newVelocity);
            }
        }
        
        public void VO_Static(bool Visualize, IList<Line> orcaLines, float ValidDist)
        {
            // my position and velocity
            Vector2 currentPos = m_position;
            Vector2 currentVelocity = m_velocity;
            
            foreach (var Other in Obstacles)
            {
                if (Other.name.Contains("start") || Other.name.Contains("goal"))
                {
                    continue;
                }
                
                
                Bounds bounds = Other.GetComponent<Renderer>().bounds;
                Vector2 otherPos = new Vector2(bounds.center.x, bounds.center.z);
                float otherRadius = 0.5f * Mathf.Abs(bounds.max.x - bounds.min.x);
                //Debug.Log(otherRadius);
                if (ValidDist > 0 && Vector2.Distance(currentPos, otherPos) > ValidDist)
                {
                    continue;
                }
                float combinedRadius = otherRadius + radius;
                Vector2 otherVelocity = new Vector2(0, 0);
                
                /* Compute everything in Velo space, draw by the origin of currentPos.*/
                //Velo Origin
                Vector2 veloOrigin = new Vector2(0, 0);
                Vector2 relativeVelocity = currentVelocity - otherVelocity;
                //cut off circle: (in Velo space)
                Vector2 relativePosition = otherPos - currentPos;
                Vector2 cutoffCenter = relativePosition / tau_obj;
                float cutoffRadius = combinedRadius / tau_obj;
                bool alreadyCovered = false;
                for (int j = 0; j < orcaLines.Count; ++j)
                {
                    if (RVOMath.det(cutoffCenter - orcaLines[j].point, orcaLines[j].direction) - cutoffRadius >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;
                        break;
                    }
                }
                if (alreadyCovered)
                {
                    continue;
                }
                //tangent points: (in Velo space)
                Vector2 tangentPosA = new Vector2();
                Vector2 tangentPosB = new Vector2();
                bool hasTangents = CircleTangents(cutoffCenter, cutoffRadius, veloOrigin, ref tangentPosA, ref tangentPosB);
                
                //// Draw the velocity obstacles of the drone, Draw everything with currentPos
                if (hasTangents && Visualize)
                {
                    DrawCircle(Vector2ToVector3(currentPos + cutoffCenter), cutoffRadius, Color.red);
                    Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(tangentPosA + currentPos), Color.blue);
                    Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(tangentPosB + currentPos), Color.blue);
                    Vector2 directionA = (tangentPosA).normalized;
                    Vector2 directionB = (tangentPosB).normalized;
                    Debug.DrawLine(Vector2ToVector3(tangentPosA + currentPos), Vector2ToVector3(tangentPosA + currentPos + 100.0f * directionA), Color.green);
                    Debug.DrawLine(Vector2ToVector3(tangentPosB + currentPos), Vector2ToVector3(tangentPosB + currentPos + 100.0f * directionB), Color.green);
                    
                    //Draw the relative velocity
                    Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(currentPos + relativeVelocity), Color.cyan);
                }
                
                // use the VO to calculate u and orcaLines
                
                float distSq = RVOMath.absSq(relativePosition);
                float radiusSq = RVOMath.sqr(radius);
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);
                
                Vector2 u;
                Line line = new Line();
                if (distSq > combinedRadiusSq)
                {
                    // w is the vector from cutoff center to relative velocity
                    Vector2 w = relativeVelocity - cutoffCenter;
                    float wLengthSq = RVOMath.absSq(w);
                    float dotProduct1 = Vector2.Dot(w, relativePosition);
                    
                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > (combinedRadiusSq * wLengthSq))
                    {
                        // project on cutoff circle
                        float wLength = RVOMath.sqrt(wLengthSq);
                        Vector2 wNormalized = w / wLength;
                        Vector2 direction = new Vector2(wNormalized.y, -wNormalized.x);
                        line.direction = direction;
                        u = (cutoffRadius - wLength) * wNormalized;
                        if (Visualize)
                        {
                            Debug.DrawLine(Vector2ToVector3(currentPos +currentVelocity), Vector2ToVector3(currentPos +currentVelocity + u), Color.yellow);
                        }
                    }
                    else
                    {
                        // project on legs
                        float leg = RVOMath.sqrt(distSq - combinedRadiusSq);
                        Vector2 direction;

                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* Project on left leg. */
                            direction = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                            line.direction = direction;
                        }
                        else
                        {
                            /* Project on right leg. */
                            direction = -new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                            line.direction = direction;
                        }

                        float dotProduct2 = Vector2.Dot(relativeVelocity, direction);
                        u = dotProduct2 * direction - relativeVelocity;
                        if (Visualize)
                        {
                            Debug.DrawLine(Vector2ToVector3(currentPos +currentVelocity), Vector2ToVector3(currentPos +currentVelocity + u), Color.yellow);
                        }
                    }
                }
                else
                {
                    float timeStep = Time.fixedDeltaTime;
                    Vector2 w = relativeVelocity - relativePosition / timeStep;
                    float wLength = RVOMath.abs(w);
                    Vector2 wNormalized = w / wLength;
                    
                    Vector2 direction = new Vector2(wNormalized.y, -wNormalized.x);
                    line.direction = direction;
                    u = (combinedRadius / timeStep - wLength) * wNormalized;
                    if (Visualize)
                    {  
                        Debug.DrawLine(Vector2ToVector3(currentPos +currentVelocity), Vector2ToVector3(currentPos +currentVelocity + u), Color.black);
                    }
                }
                
                line.point = currentVelocity + u;
                orcaLines.Add(line);
            }
        }
        
        public void VO_Dynamic(bool Visualize, IList<Line> orcaLines, float ValidDist)
        {
            // my position and velocity
            Vector2 currentPos = m_position;
            Vector2 currentVelocity = m_velocity;
            float combinedRadius = 2 * radius;
            
            foreach (var Other in Others)
            {
                Vector2 otherPos = new Vector2(Other.transform.position.x, Other.transform.position.z);
                if (ValidDist > 0 && Vector2.Distance(currentPos, otherPos) > ValidDist)
                {
                    continue;
                }
                Vector2 otherVelocity = new Vector2(Other.GetComponent<Rigidbody>().velocity.x, Other.GetComponent<Rigidbody>().velocity.z);
                
                /* Compute everything in Velo space, draw by the origin of currentPos.*/
                //Velo Origin
                Vector2 veloOrigin = new Vector2(0, 0);
                Vector2 relativeVelocity = currentVelocity - otherVelocity;
                //cut off circle: (in Velo space)
                Vector2 relativePosition = otherPos - currentPos;
                Vector2 cutoffCenter = relativePosition / tau;
                float cutoffRadius = combinedRadius / tau;
                //tangent points: (in Velo space)
                Vector2 tangentPosA = new Vector2();
                Vector2 tangentPosB = new Vector2();
                bool hasTangents = CircleTangents(cutoffCenter, cutoffRadius, veloOrigin, ref tangentPosA, ref tangentPosB);
                
                //// Draw the velocity obstacles of the drone, Draw everything with currentPos
                if (hasTangents && Visualize)
                {
                    DrawCircle(Vector2ToVector3(currentPos + cutoffCenter), cutoffRadius, Color.white);
                    Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(tangentPosA + currentPos), Color.blue);
                    Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(tangentPosB + currentPos), Color.blue);
                    Vector2 directionA = (tangentPosA).normalized;
                    Vector2 directionB = (tangentPosB).normalized;
                    Debug.DrawLine(Vector2ToVector3(tangentPosA + currentPos), Vector2ToVector3(tangentPosA + currentPos + 100.0f * directionA), Color.green);
                    Debug.DrawLine(Vector2ToVector3(tangentPosB + currentPos), Vector2ToVector3(tangentPosB + currentPos + 100.0f * directionB), Color.green);
                    
                    //Draw the relative velocity
                    Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(currentPos + relativeVelocity), Color.cyan);
                }
                
                // use the VO to calculate u and orcaLines
                
                float distSq = RVOMath.absSq(relativePosition);
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);
                
                Vector2 u;
                Line line = new Line();
                if (distSq > combinedRadiusSq)
                {
                    // w is the vector from cutoff center to relative velocity
                    Vector2 w = relativeVelocity - cutoffCenter;
                    float wLengthSq = RVOMath.absSq(w);
                    float dotProduct1 = Vector2.Dot(w, relativePosition);
                    
                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > (combinedRadiusSq * wLengthSq))
                    {
                        // project on cutoff circle
                        float wLength = RVOMath.sqrt(wLengthSq);
                        Vector2 wNormalized = w / wLength;
                        Vector2 direction = new Vector2(wNormalized.y, -wNormalized.x);
                        line.direction = direction;
                        u = (cutoffRadius - wLength) * wNormalized;
                        if (Visualize)
                        {
                            Debug.DrawLine(Vector2ToVector3(currentPos +currentVelocity), Vector2ToVector3(currentPos +currentVelocity + u), Color.yellow);
                        }
                    }
                    else
                    {
                        // project on legs
                        float leg = RVOMath.sqrt(distSq - combinedRadiusSq);
                        Vector2 direction;

                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* Project on left leg. */
                            direction = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                            line.direction = direction;
                        }
                        else
                        {
                            /* Project on right leg. */
                            direction = -new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                            line.direction = direction;
                        }

                        float dotProduct2 = Vector2.Dot(relativeVelocity, direction);
                        u = dotProduct2 * direction - relativeVelocity;
                        if (Visualize)
                        {
                            Debug.DrawLine(Vector2ToVector3(currentPos +currentVelocity), Vector2ToVector3(currentPos +currentVelocity + u), Color.yellow);
                        }
                    }
                }
                else
                {
                    float timeStep = Time.fixedDeltaTime;
                    Vector2 w = relativeVelocity - relativePosition / timeStep;
                    float wLength = RVOMath.abs(w);
                    Vector2 wNormalized = w / wLength;
                    
                    Vector2 direction = new Vector2(wNormalized.y, -wNormalized.x);
                    line.direction = direction;
                    u = (combinedRadius / timeStep - wLength) * wNormalized;
                    if (Visualize)
                    {  
                        Debug.DrawLine(Vector2ToVector3(currentPos +currentVelocity), Vector2ToVector3(currentPos +currentVelocity + u), Color.black);
                    }
                }
                
                line.point = currentVelocity + 0.5f * u;
                orcaLines.Add(line);
            }
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
        
        private Vector2 Vector3ToVector2(Vector3 v)
        {
            return new Vector2(v.x, v.z);
        }
        
        private Vector3 Vector2ToVector3(Vector2 v)
        {
            return new Vector3(v.x, 0.4f, v.y);
        }
        
        // tangents positions
        bool CircleTangents(Vector2 center, float r, Vector2 p, ref Vector2 tanPosA, ref Vector2 tanPosB) {
            p -= center;
    
            float P = p.magnitude;
    
            // if p is inside the circle, there ain't no tangents.
            if (P <= r) {
                return false;
            }
        
            float a = r * r                                          / P;    
            float q = r * (float)System.Math.Sqrt((P * P) - (r * r)) / P;
    
            Vector2 pN  = p / P;
            Vector2 pNP = new Vector2(-pN.y, pN.x);
            Vector2 va  = pN * a;
    
            tanPosA = va + pNP * q;
            tanPosB = va - pNP * q;

            tanPosA += center;
            tanPosB += center;
    
            return true;
        }
    }
}