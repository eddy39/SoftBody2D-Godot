using Godot;
using System;
using System.Collections.Generic;

public class SoftBody2DCircle : SoftBody2D_Base
{
    [Export]
    public float radius = 100f;
    public override void _Ready()
    {
        points = 12;
        base._Ready();
    }

    public override void resetBlob()
    {
        circumfrence = radius * 2.0f * (float)Math.PI ;
        area = radius * radius* (float)Math.PI ;
        areaInitial = area;
        collisionRadius = circumfrence/points * 0.5f;
        points = (int) Math.Ceiling(circumfrence/lengthSet);
        //murder collisions
        foreach(KinematicBody2D blobcol in blobCollisions) blobcol.QueueFree();
        //##### set everything empty
        //My stuff
        lenghts = new List<float>();
        blobVelocity = new List<Vector2>();
        blobCollisions = new List<KinematicBody2D>();
        changeSpring = new float[3*points];
        changePressure = new float[3*points];
        changeCollision = new float[3*points];
        changeMove = new float[3*points];
        changeStiffness = new float[3*points];
        // old stuff
        blob = new List<Vector2>();
        normals = new List<List<Vector2>>();
        // refill everything
        for (int i = 0; i < points; i++)
        {
            Vector2 delta = getVectorByAngle(radius, (float)(2*Math.PI / points) * i);
            blob.Add(delta);
            normals.Add(new List<Vector2>()); 
            normals[i].Add(delta);
            normals[i].Add(delta * (1+(1/radius))); 
            blobCollisions.Add(new KinematicBody2D());
            blobVelocity.Add(new Vector2(0,0));
            lenghts.Add(lengthSet);
        }
        lenghtsInitial = lenghts;
        blobOld = blob;
        SetCenter();
        for (int i = 0; i < points; i++) blobInitial2Center.Add(blob[i]-center);
        initBlobCollisions();
        updateSprite();
	    Update();
    }
}
