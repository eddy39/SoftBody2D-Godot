using Godot;
using System;
using System.Collections.Generic;
using System.Linq;

public class SoftBody2D : SoftBody2D_Base
{
    
    public override void _Ready()
    {
        adjustPolygon();
        base._Ready();
        
    }
    public void adjustPolygon()
    {
        poly2D = (Polygon2D) GetNode("Polygon2D");
        Vector2[] polygons =  poly2D.Polygon;
        points = polygons.Length;
        List<Vector2> newPolygons = new List<Vector2>();
        for (int i = 0; i < points; i++) curve.AddPoint(polygons[i]);
        int pointCount = curve.GetPointCount();
        for (int i = 0; i < pointCount; i++)
        {
            Vector2 spline = getSpline(i);
            curve.SetPointIn(i, -spline);
            curve.SetPointOut(i, spline);
        }
        Vector2[] bakedPoints = curve.GetBakedPoints();
        List<Vector2> drawPoints = bakedPoints.ToList();
        if (Geometry.TriangulatePolygon(bakedPoints).Length==0)
        {
            drawPoints = Geometry.ConvexHull2d(bakedPoints).ToList();
        }
        
        for (int i = 0; i < polygons.Length; i++)
        {
            newPolygons.Add(polygons[i]);

            Vector2 start = polygons[i];
            Vector2 finish = polygons[(i+1)%polygons.Length];
            float distance = start.DistanceTo(finish);
            int nPoints = (int) Math.Ceiling(distance/lengthSet)-1 ;
            Vector2 start2finish = finish-start;
            for (int j = 0; j < nPoints-1; j++)
            {
                // pick drawpoint between start and finish
                //##### quadrativ bezier
                // get third point
                Vector2 thirdPoint = (start+finish)/2 +getVectorByAngle(lengthSet,start2finish.Angle()-((float) Math.PI/2)) ;
                float x = (j+1)/((float) nPoints);
                Vector2 newPoint = thirdPoint + ((float)Math.Pow((1-x),2f))*(start-thirdPoint)+x*x*(finish-thirdPoint);
                newPolygons.Add(newPoint);
            }
        }
        poly2D.Polygon = newPolygons.ToArray();
    }

}
