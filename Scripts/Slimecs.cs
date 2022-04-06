using Godot;
using System;
using System.Collections.Generic;
public class Slimecs : Node2D
{
    public int points = 12;
    public float radius = 50.0f;
    public float circumfrenceMultiplier = 1.0f;
    public float area;
    public float circumfrence;
    public float length;
    public int iterations = 10;
    public List<Vector2> blob = new List<Vector2>();
    public List<Vector2> blobOld = new List<Vector2>();
    public Dictionary<int,float> accumulatedDisplacements = new Dictionary<int, float>();
    public List<List<Vector2>> normals = new List<List<Vector2>>();
    public Vector2 center = new Vector2(0f,0f);
    public Vector2 gravity = new Vector2(0f,30f);
    [Export]
    public float splineLength = 12f;
    [Export]
    public Curve2D curve;
    [Export]
    public Color color = new Color(0f,1f,0f,1f);
    public void verletIntegrate(int i, float _delta)
    {
        Vector2 temp = blob[i];
        blob[i] = (blob[i] + (blob[i] - blobOld[i]));
        blobOld[i] = temp; 
    }
    public Vector2 setDistance(Vector2 currentPoint, Vector2 anchor, float distance)
    {
        Vector2 toAnchor = currentPoint - anchor;
        toAnchor = toAnchor.Normalized() * distance;
        return toAnchor+anchor;
    }
    public override void _Ready()
    {
        circumfrence = radius * 2.0f * (float)Math.PI * circumfrenceMultiplier;
        length = circumfrence * 1.15f / (float) points;
        area =radius * radius* (float)Math.PI;
        resetBlob();
    }
    public Vector2 findCentroid()
    {
        float x = 0f;
        float y = 0f;
        for (int i = 0; i < points; i++)
        {
            x+=blob[i].x;
            y+=blob[i].y;
        }
        Vector2 cent = new Vector2();
        cent.x = x/((float) points);
        cent.y = y/((float) points);
        return cent;
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
        //adjusted stuff(might be wrong)
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
        for (int i = 0; i < points; i++)
        {
            DrawLine(normals[i][0],normals[i][1], new Color(1f,0f,0f),3);
            DrawLine(blob[i], blob[(i + 1) % points], new Color(0f,0.5f,1f), 10);
        }
        Vector2[] bakedPoints = curve.GetBakedPoints();
        Vector2[] drawPoints = bakedPoints;
        // adjusted (might not work)
        if (Geometry.TriangulatePolygon(bakedPoints).Length==0)
        {
            drawPoints = Geometry.ConvexHull2d(bakedPoints);
        }
        
        //DrawPolyline(drawPoints, new Color(0f,0f,0f), 2.0f, true);
        DrawPolygon(drawPoints,new Color[] { color });
    }
    
    public Vector2 getVectorByLA (float length, float angle)
    {
        //angle;
        Vector2 vector = new Vector2();
        vector.x = (float) Math.Cos((double)angle);
        vector.y = (float) Math.Sin((double)angle);
        vector *= length;
        //GD.Print(vector);
        return vector;
    }
	
    public override void _Process(float delta)
    {
        for (int i = 0; i < points; i++)
        {
            //verletIntegrate(i,delta); //makes things blobby
            blob[i]+= gravity * delta;
        }
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int i = 0; i < points; i++)
            {
                Vector2 segment = blob[i];
                int nextIndex = i+1;
                if (i == points-1) nextIndex = 0;
                
                Vector2 nextSegment = blob[nextIndex];
                Vector2 toNext = segment-nextSegment;
                if (toNext.Length()>length)
                {
                    toNext = toNext.Normalized() * length;
                    Vector2 offset = (segment - nextSegment) - toNext;
                    accumulatedDisplacements[i*3]-= offset.x / 2f;
                    accumulatedDisplacements[i*3 +1]-= offset.y / 2f;
                    accumulatedDisplacements[i*3 +2] += 1f;

                    accumulatedDisplacements[nextIndex*3]+= offset.x / 2f;
                    accumulatedDisplacements[nextIndex*3 +1]+= offset.y / 2f;
                    accumulatedDisplacements[nextIndex*3 +2]+= 1f;
                }
            }
            float deltaArea = 0;
            float curArea = getCurArea();
            if (curArea<area*2) deltaArea = area- curArea;

            float dilationDistance = deltaArea/circumfrence;
            for (int i = 0; i < points; i++)
            {
                int prevIndex = i-1;
                if (i==0) prevIndex = points-1;
                int nextIndex = i+1;
                if (i == points-1) nextIndex = 0;
                Vector2 normal = blob[nextIndex]-blob[prevIndex];
                float angle = normal.Angle()-(float) (Math.PI/2); // adjusted
                normal = getVectorByLA(1f,angle);
                normals[i][0] = blob[i];
                normals[i][1] = blob[i] + (normal * 200.0f);
                accumulatedDisplacements[(i * 3)] += normal.x * dilationDistance;
                accumulatedDisplacements[(i * 3) + 1] += normal.y * dilationDistance;
                accumulatedDisplacements[(i * 3) + 2] += 1.0f;
            }
            for (int i = 0; i < points; i++)
            {
                if (accumulatedDisplacements[(i * 3) + 2] > 0)
                {
                    blob[i] += new Vector2(accumulatedDisplacements[(i * 3)], accumulatedDisplacements[(i * 3) + 1]) / accumulatedDisplacements[(i * 3) + 2];
                }
            }
            for (int i = 0; i < points*3; i++) accumulatedDisplacements[i] = 0;
            
            for (int i = 0; i < points; i++)
            {
                if (Input.IsActionPressed("leftClick") && ((blob[i] - GetGlobalMousePosition()).Length() < 40.0f))
				{
                    blob[i] = setDistance(blob[i], GetGlobalMousePosition(), 40.0f);
			    }
                if (blob[i].Length() > 175.0f)
                {        
                    blob[i] = setDistance(blob[i], new Vector2(0.0f, 0.0f), 175.0f);
                }
            }
        }
        updateSprite();
	    Update();
    }
    public void resetBlob()
    {
        blob = new List<Vector2>();
        blobOld = new List<Vector2>();
        accumulatedDisplacements = new Dictionary<int, float>();
        normals = new List<List<Vector2>>();
        for (int i = 0; i < points; i++)
        {
            Vector2 delta = getVectorByLA(radius, (float)(2*Math.PI / points) * i);
            blob.Add(Position + delta);
            blobOld.Add(Position + delta);
            normals.Add(new List<Vector2>()); // might cause problems
            normals[i].Add(Position + delta);
            normals[i].Add(Position + delta * 1.5f);
        }
        for (int i = 0; i < points*3; i++) accumulatedDisplacements[i] = 0.0f;
        updateSprite();
	    Update();
    }
    
    // Circumfrence Slider
    public void _on_HSlider_value_changed(float value)
    {
        circumfrenceMultiplier = value;
        circumfrence = radius * 2.0f * (float)Math.PI * circumfrenceMultiplier;
	    length = circumfrence * 1.15f / (float) points;
    }
    public void  _on_HSlider2_value_changed(float value)
    {
        gravity.y = value;
    }
    public void _on_HSlider3_value_changed(float value)
    {
        radius = value;
        area = radius * radius * (float)Math.PI;
        circumfrence = radius * 2.0f * (float)Math.PI * circumfrenceMultiplier;
        length = circumfrence * 1.15f / (float) points;
    }
    public void _on_HSlider4_value_changed(float value)
    {
        points = (int) value;
        area = radius * radius * (float)Math.PI;
        circumfrence = radius * 2.0f * (float)Math.PI * circumfrenceMultiplier;
        length = circumfrence * 1.15f / (float) points;
        
        resetBlob();
    }

}   