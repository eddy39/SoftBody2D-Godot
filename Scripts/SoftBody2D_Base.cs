using Godot;
using System;
using System.Collections.Generic;

public class SoftBody2D_Base : Node2D
{
    public int points=2;
    public float area;
    public float areaInitial;
    public float circumfrence;
    public List<float> lenghts = new List<float>();
    public List<float> lenghtsInitial = new List<float>();
    
    public List<Vector2> blob = new List<Vector2>();
    public List<Vector2> blobOld = new List<Vector2>();
    public List<Vector2> blobInitial2Center = new List<Vector2>();
    
    public float[] changeSpring;
    public float[] changePressure;
    public float[] changeCollision;
    public float[] changeMove;
    public float[] changeStiffness;
    public List<List<Vector2>> normals = new List<List<Vector2>>();
    public List<KinematicBody2D> blobCollisions = new List<KinematicBody2D>();
    public List<Vector2> blobVelocity = new List<Vector2>();
    public Vector2 moveTo;
    public bool moving=false;
    public float speed = 100;
    public float acceleration = 20;
    [Export]
    public float springFactor = 30;
    [Export]
    public float pressureFactor = 50;
    [Export]
    public float stiffnessFactor = 50f;
    [Export]
    public float collisionFactor = 0;
    
    public float collisionRadius;
    public float splineLength = 12f;
    public Curve2D curve = new Curve2D();    
    [Export]
    public Polygon2D poly2D;
    public Polygon2D poly2DInitial;
    [Export]
    public bool moveRel = true;
    public Vector2 center;
    public bool allowPhysics = false;
    [Export]
    public float lengthSet=20;
    public Vector2[] helper;
    
    public override void _Ready()
    {
        // calculate stuff
        
        poly2D = (Polygon2D) GetNode("Polygon2D");
        poly2DInitial = poly2D;
        // reset everything
        resetBlob();
        helper = new Vector2[points];
        allowPhysics=true;
    }
    public void verletIntegrate(int i, float _delta)
    {
        Vector2 temp = blob[i];
        blob[i] = (blob[i] + (blob[i] - blobOld[i]));
        blobOld[i] = temp; 
    }
    public Vector2 getPoint(int i)
    {
        int pointCount = curve.GetPointCount();
        if (i<0) 
        {
            i = pointCount-1 +i;
        } else {
            i = i%(pointCount-1);
        }
        return curve.GetPointPosition(i);
    }
    public Vector2 getSpline(int i)
    {
        Vector2 lastPoint = getPoint(i-1);
        Vector2 nextPoint = getPoint(i+1);
        Vector2 spline = lastPoint.DirectionTo(nextPoint)*splineLength;
        return spline;
    }
    public void updateSprite()
    {
        curve.ClearPoints();
        for (int i = 0; i < points; i++) curve.AddPoint(blob[i]);
        curve.AddPoint(blob[0]);

        int pointCount = curve.GetPointCount();
        for (int i = 0; i < pointCount; i++)
        {
            Vector2 spline = getSpline(i);
            curve.SetPointIn(i, -spline);
            curve.SetPointOut(i, spline);
        }
    }
    public void updateCollision()
    {
        for (int i = 0; i < points; i++)
        {
            CollisionShape2D colShape = (CollisionShape2D) blobCollisions[i].GetChild(0);
            Vector2 normal = (normals[i][0]-normals[i][1]);
            colShape.Position = normal*collisionRadius/normal.Length();
        }
        
    }
    public float getCurArea()
    {
        float area = 0f;
        int j = points -1;
        for (int i = 0; i < points; i++)
        {
            area +=( blob[j].x + blob[i].x) * (blob[j].y - blob[i].y);
            j=i;
        }
        return Math.Abs(area/2);
    }
    public override void _Draw()
    {
        
        Vector2[] bakedPoints = curve.GetBakedPoints();
        Vector2[] drawPoints = bakedPoints;
        if (Geometry.TriangulatePolygon(bakedPoints).Length==0)
        {
            drawPoints = Geometry.ConvexHull2d(bakedPoints);
        }
        
        //DrawPolyline(drawPoints, new Color(0f,0f,0f), 2.0f, true);
        poly2D.Polygon = drawPoints;
        poly2D.Update();
        //DrawPolygon(drawPoints,new Color[] { color });
        DrawCircle(moveTo,5,new Color(1,0,0));
        DrawCircle(center,5,new Color(1,1,0.5f));
        for (int i = 0; i < points; i++)
        {
            //DrawLine(normals[i][0],normals[i][1], new Color(1f,0f,0f),3);
            //DrawLine(blob[i], blob[(i + 1) % points], new Color(0f,0.5f,1f), 10);
            //DrawLine(blob[i],blob[i]+(helper[i]*50), new Color(1f,0f,0f),3);
            
        }
    }
    
    public Vector2 getVectorByAngle (float length, float angle)
    {
        Vector2 vector = new Vector2();
        vector.x = (float) Math.Cos((double)angle);
        vector.y = (float) Math.Sin((double)angle);
        vector *= length;
        return vector;
    }

    public override void _PhysicsProcess(float delta)
    {
        base._PhysicsProcess(delta);
        if(!allowPhysics) return;
        //#################### Calculate changes in outer springs
        for (int i = 0; i < points; i++)
        {
            //verletIntegrate(i,delta);
            int nextIndex = (i+1)%points;

            Vector2 segment = blob[i];
            Vector2 nextSegment = blob[nextIndex];
            
            Vector2 Distance = segment-nextSegment;
            if (Distance.Length()>lenghts[i]*1.2f)// if things are too far apart pull back together
            {
                Vector2 change = Distance.Normalized() * (Distance.Length()-lenghts[i]);// setting Distance back to length
                changeSpring[i*3]-= change.x / 2f  ; //x
                changeSpring[i*3 +1]-= change.y / 2f  ; //y
                changeSpring[i*3 +2] += .5f; //weight

                changeSpring[nextIndex*3]+= change.x / 2f ;//x
                changeSpring[nextIndex*3 +1]+= change.y / 2f ;//y
                changeSpring[nextIndex*3 +2]+= .5f; //weight
            } else if (Distance.Length()<lenghts[i]*0.5f){
                Vector2 change = Distance.Normalized() * (Distance.Length()-lenghts[i]);// setting Distance back to length
                changeSpring[i*3]-= change.x / 2f  ; //x
                changeSpring[i*3 +1]-= change.y / 2f  ; //y
                changeSpring[i*3 +2] += .5f; //weight

                changeSpring[nextIndex*3]+= change.x / 2f ;//x
                changeSpring[nextIndex*3 +1]+= change.y / 2f ;//y
                changeSpring[nextIndex*3 +2]+= .5f; //weight
            }
        }
        //#################### Calculate changes in pressure
        float deltaArea = area- getCurArea();
        float dilationDistance = 0;
        if (Math.Abs(deltaArea/area)>0.01) dilationDistance = deltaArea/area;
        
        for (int i = 0; i < points; i++) 
        {
            int prevIndex = i-1;
            if (i==0) prevIndex = points-1;
            int nextIndex = i+1;
            if (i == points-1) nextIndex = 0;
            // calculate normal vector
            Vector2 normal = blob[nextIndex]-blob[prevIndex];
            float angle = normal.Angle()-(float) (Math.PI/2);
            normal = getVectorByAngle(1f,angle);
            normals[i][0] = blob[i];
            normals[i][1] = blob[i] + (50*normal);
            changePressure[(i * 3)] += normal.x * dilationDistance ; //x 
            changePressure[(i * 3) + 1] += normal.y * dilationDistance ; //y
            changePressure[(i * 3) + 2] += 1f; //weight
            //if (deltaArea!=0) lenghts[i]= lenghtsInitial[i]; // reset length to initial length
        }
        // ############ calculate changes for stiffness
        for (int i = 0; i < points; i++) 
        {
            Vector2 change= (center+blobInitial2Center[i])-blob[i];
            
            changeStiffness[(i * 3)] += change.x ; //x 
            changeStiffness[(i * 3) + 1] += change.y ; //y
            changeStiffness[(i * 3) + 2] += 1f; //weight
            
        }
        // ############ calculate changes for Collision
        for (int i = 0; i < points; i++)
        {
            
            if ((blobCollisions[i].GetSlideCount()>0))
            {
                blob[i] = blobCollisions[i].Position;
                Vector2 moveNormal = new Vector2(0,0);
                for (int j = 0; j < blobCollisions[i].GetSlideCount(); j++)
                {
                    KinematicCollision2D collision = blobCollisions[i].GetSlideCollision(j);
                    moveNormal +=collision.Normal;
                    //moving = false; // set moving false on any collision
                }
                
                Vector2 change = moveNormal;
                changeCollision[(i * 3)] += moveNormal.x; //x
                changeCollision[(i * 3) + 1] += moveNormal.y; //y
                changeCollision[(i * 3) + 2] += 1f; //weight

                helper[i] = change;
                
            }
        }
        if (moving) // move to Target
        {
            for (int i = 0; i < points; i++) 
            {
                if (center.DistanceTo(moveTo)<20)// if one point close enough stop
                {
                    moving=false;
                    break;
                }
                if (blobVelocity[i].Length()>speed) continue; // if fast enough skip
                //Vector2 direction = moveTo-blob[i];//
                Vector2 direction = new Vector2();
                if (moveRel) 
                {
                    direction = moveToRel(i);
                } else {
                    direction = moveToAbs(i);
                }
                Vector2 change = direction.Normalized();
                changeMove[(i * 3)] += change.x ; //x
                changeMove[(i * 3) + 1] += change.y; //y
                changeMove[(i * 3) + 2] += 1f; //weight
            }
        }
        /* if (!moving)// if at target slow down/stop
        {
            for (int i = 0; i < points; i++)
            {
                Vector2 change = -blobVelocity[i];// counter all current velocity
                // set lengths to current length
                int nextIndex = i+1;
                if (i == points-1) nextIndex = 0;
                lenghts[i] = (lenghts[i]+(blob[i]-blob[nextIndex]).Length())/2;
            }
        } */
        // apply all changes
        for (int i = 0; i < points; i++)
        {
            Vector2 appliedChange = new Vector2(0,0);
            //### Move
            if (changeMove[(i * 3) + 2] > 0)
            {
                appliedChange += new Vector2(changeMove[(i * 3)], changeMove[(i * 3) + 1]) * changeMove[(i * 3) + 2] * acceleration;
            }
            //### Collision
            if (changeCollision[(i * 3) + 2] > 0)
            {
                appliedChange += new Vector2(changeCollision[(i * 3)], changeCollision[(i * 3) + 1]) * changeCollision[(i * 3) + 2] * collisionFactor;
            }
            //### Pressure
            if (changePressure[(i * 3) + 2] > 0)
            {
                appliedChange += new Vector2(changePressure[(i * 3)], changePressure[(i * 3) + 1]) * changePressure[(i * 3) + 2] * pressureFactor;
            }
            //### Stiffness
            if (changeStiffness[(i * 3) + 2] > 0)
            {
                appliedChange += new Vector2(changeStiffness[(i * 3)], changeStiffness[(i * 3) + 1]) * changeStiffness[(i * 3) + 2] * stiffnessFactor;
            }
            //### Spring
            if (changeSpring[(i * 3) + 2] > 0)
            {
                appliedChange += new Vector2(changeSpring[(i * 3)], changeSpring[(i * 3) + 1]) * changeSpring[(i * 3) + 2] * springFactor;
            }

            if (changeCollision[(i * 3) + 2]>0) // indicates whether Collision happened
            {   // collision case
                blobVelocity[i] = blobCollisions[i].MoveAndSlide(appliedChange);
                blob[i] = blobCollisions[i].Position;
            } else {
                // no collision case
                blobVelocity[i] = blobCollisions[i].MoveAndSlide(appliedChange* delta);
                blob[i] += appliedChange * delta;
                blobCollisions[i].Position =  blob[i];
            }
            
        }
        // Reset all changes
        changeSpring = new float[3*points];
        changePressure = new float[3*points];
        changeStiffness = new float[3*points];
        changeCollision = new float[3*points];
        changeMove = new float[3*points];
        
        // Update
        SetCenter();
        updateCollision();
        updateSprite();
	    Update();
    }
    public List<Vector2> calculateNormal(int i)
    {
        // calculate normal vector
        int prevIndex = i-1;
        if (i==0) prevIndex = points-1;
        int nextIndex = i+1;
        if (i == points-1) nextIndex = 0;
        Vector2 normal = blob[nextIndex]-blob[prevIndex];
        float angle = normal.Angle()-(float) (Math.PI/2);
        normal = getVectorByAngle(1f,angle);
        List<Vector2>returnNormals = new List<Vector2>();
        returnNormals.Add(blob[i]);
        returnNormals.Add(blob[i] + (normal));
        return returnNormals;
    }
    public virtual void resetBlob()
    {
        allowPhysics = false;
        //
        //murder collisions
        foreach(KinematicBody2D blobcol in blobCollisions) blobcol.QueueFree();
        //##### set everything empty
        //My stuff
        Vector2[] polygons =  poly2DInitial.Polygon;
        points = polygons.Length;
        lenghts = new List<float>();
        blobVelocity = new List<Vector2>();
        blobCollisions = new List<KinematicBody2D>();
        changeSpring = new float[3*points];
        changePressure = new float[3*points];
        changeCollision = new float[3*points];
        changeMove = new float[3*points];
        changeStiffness = new float[3*points];
        helper = new Vector2[points];
        // old stuff
        blob = new List<Vector2>();
        blobOld = new List<Vector2>();
        normals = new List<List<Vector2>>();
        // Refill everything
       
        circumfrence = 0;
        lenghts = new List<float>();
        for (int i = 0; i < points; i++)
        {
            lenghts.Add((polygons[i]-polygons[(i+1)%points]).Length());
            circumfrence+=lenghts[i];
            blob.Add(polygons[i]);
            blobCollisions.Add(new KinematicBody2D());
            blobVelocity.Add(new Vector2(0,0));
        }
        for (int i = 0; i < points; i++) normals.Add(calculateNormal(i)); // has to be done after all blobs 
        lenghtsInitial= lenghts;
        blobOld = blob;
        area = getCurArea();
        areaInitial = area;
        collisionRadius = circumfrence/points * 0.5f;
        SetCenter();
        for (int i = 0; i < points; i++) blobInitial2Center.Add(blob[i]-center);

        initBlobCollisions();
        updateSprite();
	    Update();
        allowPhysics = true;
    }
    public virtual Vector2 moveToAbs(int i )
    {
        return moveTo-blob[i];
    }
    public virtual Vector2 moveToRel(int i )
    {
        return moveTo-center;
    }
    public void SetCenter()
    {
        Vector2 direction = new Vector2(0,0);
        for (int i = 0; i < points; i++)
        {
            direction+=blob[i];
        }
        center = direction/points;
    }
    
    public void initBlobCollisions()
    {
        for (int i = 0; i < points; i++)
        {
            CollisionShape2D colShape = new CollisionShape2D();
            CircleShape2D circleShape = new CircleShape2D();
            circleShape.Radius = collisionRadius;
            colShape.Shape = circleShape;
            //normals are 1 radius long
            colShape.Position = (normals[i][0]-normals[i][1])*collisionRadius;
            blobCollisions[i].AddChild(colShape);
            blobCollisions[i].Position = blob[i];
            for (int j = 0; j < points; j++)
            {
                blobCollisions[i].AddCollisionExceptionWith(blobCollisions[j]);
            }

            AddChild(blobCollisions[i]);
        }
    }    
}
