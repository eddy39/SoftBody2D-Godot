#include "Softbody2DPolygon.h"

using namespace godot;

void Softbody2DPolygon::_register_methods()
{
    register_method("_ready",&Softbody2DPolygon::_ready);
    register_method("_physics_process",&Softbody2DPolygon::_physics_process);
    register_method("_draw",&Softbody2DPolygon::_draw);
    register_method("on_body_entered",&Softbody2DPolygon::on_body_entered);
    register_method("on_body_exited",&Softbody2DPolygon::on_body_exited);
    register_method("SoftbodyPhysics",&Softbody2DPolygon::SoftbodyPhysics);
    register_method("HardbodyPhysics",&Softbody2DPolygon::HardbodyPhysics);
    register_method("move_and_slide",&Softbody2DPolygon::move_and_slide);

    register_property("moveTo",&Softbody2DPolygon::moveTo,Vector2(0,0));
    register_property("moving",&Softbody2DPolygon::moving,false);
    register_property("allowPhysics",&Softbody2DPolygon::allowPhysics,false);
    register_property("moveRel",&Softbody2DPolygon::moveRel,false);
    register_property("useSoftbody",&Softbody2DPolygon::useSoftbody,true);
    register_property("center",&Softbody2DPolygon::center,Vector2(0,0));


    register_property("acceleration",&Softbody2DPolygon::acceleration,80.0f);
    register_property("springFactor",&Softbody2DPolygon::springFactor,20.0f);
    register_property("pressureFactor",&Softbody2DPolygon::pressureFactor,20.0f);
    register_property("stiffnessFactor",&Softbody2DPolygon::stiffnessFactor,10.0f);
    register_property("collisionFactor",&Softbody2DPolygon::collisionFactor,50.0f);

    register_property("moveDecay",&Softbody2DPolygon::moveDecay,0.0f);
    register_property("springDecay",&Softbody2DPolygon::springDecay,0.0f);
    register_property("pressureDecay",&Softbody2DPolygon::pressureDecay,0.0f);
    register_property("stiffnessDecay",&Softbody2DPolygon::stiffnessDecay,0.0f);
    register_property("collisionDecay",&Softbody2DPolygon::collisionDecay,0.0f);

    register_property("useSoftbody",&Softbody2DPolygon::useSoftbody,true);
    register_property("createObserverarea",&Softbody2DPolygon::createObserverarea,false);

    register_property("area",&Softbody2DPolygon::area,1.0f);
    register_property("areaInitial",&Softbody2DPolygon::areaInitial,1.0f);

    register_property("lengthSet",&Softbody2DPolygon::lengthSet,40.0f);
}

Softbody2DPolygon::Softbody2DPolygon()
{

}
Softbody2DPolygon::~Softbody2DPolygon()
{

}

Vector2 Softbody2DPolygon::getPoint(int i)
{
    int64_t pointCount = curve->get_point_count();
    if (i<0)
    {
        i = pointCount-1 +i;
    } else
    {
        i = i%(pointCount-1);
    }
    return curve->get_point_position(i);
}
Vector2 Softbody2DPolygon::getSpline(int i)
{
    Vector2 lastPoint = getPoint(i-1);
    Vector2 nextPoint = getPoint(i+1);
    return lastPoint.direction_to(nextPoint)*splineLength;
}

void Softbody2DPolygon::updateSprite()
{
    curve->clear_points();
    for (int i = 0; i < points; i++)
    {
        curve->add_point(blob[i]);
    }
    curve->add_point(blob[0]);
    int64_t pointCount = curve->get_point_count();
    for (int i = 0; i < pointCount; i++)
    {
        Vector2 spline = getSpline(i);
        curve->set_point_in(i,-spline);
        curve->set_point_out(i,spline);
    }
}
void Softbody2DPolygon::initBlobCollisions()
{
    for (int i = 0; i < points; i++)
    {
        CollisionShape2D* colShape = CollisionShape2D::_new();
        Ref<CircleShape2D> circleShape = Ref<CircleShape2D>(CircleShape2D::_new());
        circleShape -> set_radius(collisionRadius);
        colShape -> set_shape(circleShape);
        colShape -> set_position((-normals[i])*collisionRadius);
        blobCollisions[i]->add_child(colShape);
        blobColShapes.push_back(colShape);
        for (int j = 0; j < points; j++) blobCollisions[i]->add_collision_exception_with(blobCollisions[j]);
        
        blobCollisions[i]->set_name("SoftbodyBorderCollisionBody_"+blobCollisions[i]->get_name());
        add_child(blobCollisions[i]);
    }
}
void Softbody2DPolygon::initObserverArea()
{
    ObserverArea = Area2D::_new();
    AreaShape = CollisionPolygon2D::_new();
    PoolVector2Array AreaPolygon = PoolVector2Array();
    for (int i = 0; i < points; i++) AreaPolygon.append(blob[i]+normals[i]*collisionRadius*2);
    AreaShape-> set_polygon(AreaPolygon);
    ObserverArea ->add_child(AreaShape);
    add_child(ObserverArea);
    observerareaExists=true;
    
}

void Softbody2DPolygon::on_body_entered(Variant body)
{
    body_count+=1;
    if (body_count>blobCollisions.size()) useSoftbody = true;
}
void Softbody2DPolygon::on_body_exited(Variant body)
{
    body_count-=1;
    if (body_count<=blobCollisions.size()) useSoftbody = false;
}
void Softbody2DPolygon::SetCenter()
{
    Vector2 direction = Vector2(0,0);
    
    for (int i = 0; i < points; i++) direction+=blob[i];
    
    centerChange = (direction/points) - center;
    center = direction/points;        
}
float Softbody2DPolygon::getCurArea()
{
    float area_ = 0;
    int j = points -1;
    for (int i = 0; i < points; i++)
    {
        area_ +=( blob[j].x + blob[i].x) * (blob[j].y - blob[i].y);
        j=i;
    }
    return abs(area_/2);
}
Vector2 Softbody2DPolygon::getVectorByAngle(float length, float angle)
{
    Vector2 vector = Vector2(0,0);
    vector.x = cos(angle);
    vector.y = sin(angle);
    vector *= length;
    return vector;
}
Vector2 Softbody2DPolygon::calculateNormal(int i)
{
    int prevIndex = i-1;
    if (i==0) prevIndex = points-1;
    int nextIndex = (i+1);
    if (i==points-1) nextIndex = 0;
    Vector2 normal = blob[nextIndex]-blob[prevIndex];
    float angle = normal.angle()-(Math_PI/2);
    normal = getVectorByAngle(1,angle);
    
    return normal;
    
}
void Softbody2DPolygon::resetBlob()
{
    allowPhysics = false;
    for (size_t i = 0; i < blobCollisions.size(); i++)
    {
        blobCollisions[i]->queue_free();
    }
    PoolVector2Array polygons = polygonInitial;
    points = polygons.size();
    lengths = std::vector<float>();
    blobCollisions = std::vector<KinematicBody2D*>();
    changeSpring = std::vector<Vector2>();
    changePressure = std::vector<Vector2>();
    changeCollision = std::vector<Vector2>();
    changeMove = std::vector<Vector2>();
    changeStiffness = std::vector<Vector2>();

    blob = std::vector<Vector2>();
    normals = std::vector<Vector2>();
    circumfrence = 0;
    for (size_t i = 0; i < points; i++)
    {
        lengths.push_back((polygons[i]-polygons[(i+1)%points]).length());
        circumfrence+=lengths.back();
        blob.push_back(polygons[i]);
        blobCollisions.push_back(KinematicBody2D::_new());
        changeSpring.push_back(Vector2(0,0));
        changePressure.push_back(Vector2(0,0));
        changeCollision.push_back(Vector2(0,0));
        changeMove.push_back(Vector2(0,0));
        changeStiffness.push_back(Vector2(0,0));

    }
    for (size_t i = 0; i < points; i++)
    {
        normals.push_back(calculateNormal(i));
    }
    lengthsInitial = lengths;
    area = getCurArea();
    areaInitial = area;
    collisionRadius = circumfrence/points *.5;
    // move center to 0,0
    SetCenter();
    for (int i = 0; i < points; i++) blob[i]-=center;
    SetCenter();
    centerChange=Vector2(0,0);// reset centerChange
    // set blobInitial2Center (for stiffness)
    for (int i = 0; i < points; i++) blobInitial2Center.push_back(blob[i]-center);
    
    initBlobCollisions();
    if (createObserverarea) initObserverArea();
    
    updateCollision();
    updateSprite();
    update();
}

void Softbody2DPolygon::adjustPolygon()
{
    PoolVector2Array polygons = get_polygon();
    points = polygons.size();
    PoolVector2Array newPolygons = PoolVector2Array();
    for (int i = 0; i < points; i++) curve->add_point(polygons[i]);
    int64_t pointCount = curve->get_point_count();
    for (int i = 0; i < pointCount; i++)
    {
        Vector2 spline = getSpline(i);
        curve->set_point_in(i, -spline);
        curve->set_point_out(i, spline);
    }
    PoolVector2Array bakedPoints = curve->get_baked_points();
    PoolVector2Array drawPoints = bakedPoints;
    if (geometryObj->triangulate_polygon(bakedPoints).size()==0)
    {
        drawPoints = geometryObj->convex_hull_2d(bakedPoints);
    }
    for (int i = 0; i < polygons.size(); i++)
    {
        newPolygons.push_back(polygons[i]);

        Vector2 start = polygons[i];
        Vector2 finish = polygons[(i+1)%polygons.size()];
        float distance = start.distance_to(finish);
        int nPoints = ceil(distance/lengthSet)-1 ;
        Vector2 start2finish = finish-start;
        if (nPoints<=0) continue;
        for (int j = 0; j < nPoints-1; j++)
        {
            // pick drawpoint between start and finish
            //##### quadrativ bezier
            // get third point
            Vector2 thirdPoint = (start+finish)/2 +getVectorByAngle(lengthSet,start2finish.angle()-((float) Math_PI/2)) ;
            float x = (j+1)/((float) nPoints);
            Vector2 newPoint = thirdPoint + ((float)pow((1-x),2))*(start-thirdPoint)+x*x*(finish-thirdPoint);
            newPolygons.push_back(newPoint);
        }
    }
    set_polygon(newPolygons);
}

void Softbody2DPolygon::_init()
{
}


void Softbody2DPolygon::_ready()
{
    if (acceleration==0)  acceleration=acceleration_;
    if (springFactor==0)  springFactor=springFactor_;
    if (pressureFactor==0)  pressureFactor=pressureFactor_;
    if (collisionFactor==0)  collisionFactor=collisionFactor_;
    if (stiffnessFactor==0)  stiffnessFactor=stiffnessFactor_;

    if (moveDecay==0)  moveDecay=moveDecay_;
    if (springDecay==0)  springDecay=springDecay_;
    if (pressureDecay==0)  pressureDecay=pressureDecay_;
    if (collisionDecay==0)  collisionDecay=collisionDecay_;
    if (stiffnessDecay==0)  stiffnessDecay=stiffnessDecay_;
    
    if (lengthSet==0)  lengthSet=lengthSet_;

    if (useSoftbody==false) useSoftbody=true;
    if (allowPhysics==false) allowPhysics=true;
    

    curve = Curve2D::_new();
    adjustPolygon();
    polygonInitial = get_polygon();
    prnt = Object::cast_to<Node2D>(get_parent());
    resetBlob();
    
    allowPhysics=true;
}
Vector2 Softbody2DPolygon::moveToAbs(int i )
    {
        return moveTo-blob[i];
    }
Vector2 Softbody2DPolygon::moveToRel(int i )
{
    return moveTo-center;
}
void Softbody2DPolygon::updateCollision()
{
    for (int i = 0; i < points; i++)
    {
        blobColShapes[i]->set_position(-normals[i]*collisionRadius/normals[i].length());
    }
   
}
void Softbody2DPolygon::updateObserverArea()
{
   
    PoolVector2Array AreaPolygon = PoolVector2Array();
    for (int i = 0; i < points; i++) AreaPolygon.append(blob[i]+normals[i]*collisionRadius*2);
    AreaShape-> set_polygon(AreaPolygon);

}

void Softbody2DPolygon::_physics_process(float delta)
{
    if (!allowPhysics) return;
    if (useSoftbody) {
        SoftbodyPhysics(delta);
    } else {
        HardbodyPhysics(delta);
    }

}
void Softbody2DPolygon::SoftbodyPhysics(float delta)
{
    collisionHappened = false;

    for (int i = 0; i < points; i++)
    {
        //#################### Calculate changes in outer springs
        int nextIndex = (i+1)%points;
        Vector2 change;
        Vector2 Distance = blob[i]-blob[nextIndex];
        if (Distance.length()>lengths[i]*1.2)// if things are too far apart pull back together
        {
            change = Distance.normalized() * (Distance.length()-lengths[i])/ 2 *springFactor;// setting Distance back to length
            changeSpring[i]-= change   ;
            changeSpring[nextIndex]+= change  ;
            
        } else if (Distance.length()<lengths[i]*0.5){
            change = Distance.normalized() * (Distance.length()-lengths[i])/ 2 *springFactor;// setting Distance back to length
            changeSpring[i]-= change ;
            changeSpring[nextIndex]+= change  ;
        }
        

        // ############ calculate changes for stiffness
        
        change= (center+blobInitial2Center[i])-blob[i];
        changeStiffness[i] += change*stiffnessFactor ;
        // ############ calculate changes for Collision
        
        if ((blobCollisions[i] -> get_slide_count()>0))
        {
            blob[i] = blobCollisions[i] -> get_position();
            Vector2 moveNormal = Vector2(0,0);
            for (int j = 0; j < blobCollisions[i] -> get_slide_count(); j++)
            {
                Ref<KinematicCollision2D> collisionRef = blobCollisions[i] -> get_slide_collision(j);
                moveNormal +=collisionRef -> get_normal();
                //moving = false; // set moving false on any collision
            }
            changeCollision[i] += moveNormal*collisionFactor;
            collisionHappened = true;
        }

        if (moving) // move to Target
        {
            if (center.distance_to(moveTo)<20)// if one point close enough stop
            {
                moving=false;
                break;
            }
            Vector2 direction = Vector2(0,0);
            if (moveRel) 
            {
                direction = moveToRel(i);
            } else {
                direction = moveToAbs(i);
            }
            change = direction.normalized();
            changeMove[i] += change*acceleration ; 
        }    
    }
    //#################### Calculate changes in pressure
    float deltaArea = area- getCurArea();
    float dilationDistance = deltaArea/area;
        
    if (abs(dilationDistance)>0.01) 
    {
        for (int i = 0; i < points; i++)
        {
            int prevIndex = i-1;
            if (i==0) prevIndex = points-1;
            int nextIndex = (i+1)%points;
            // calculate normal vector
            Vector2 normal = blob[nextIndex]-blob[prevIndex];
            float angle = normal.angle()-(float) (Math_PI/2);
            normal = getVectorByAngle(1,angle);
            normals[i] = normal;
            changePressure[i] += normal * dilationDistance *pressureFactor;
        }
    }

    // apply all changes
    for (int i = 0; i < points; i++)
    {
        Vector2 appliedChange = Vector2(0,0);
        
        //### Move
        appliedChange+=changeMove[i];
        //### Pressure
        appliedChange+=changePressure[i];
        //### Stiffness
        appliedChange+=changeStiffness[i];
        //### Spring
        appliedChange+=changeSpring[i];
        //### Collision
        appliedChange+=changeCollision[i];
        //### if close to no change skip
        if (collisionHappened) // 
        {   // collision case
            blobCollisions[i] -> move_and_slide(appliedChange);
            blob[i] = blobCollisions[i] -> get_position();
        } else {
            // no collision case
            blobCollisions[i] -> move_and_slide(appliedChange* 0.001);
            blob[i] += appliedChange * delta; //replace delta with time_passed
            blobCollisions[i] -> set_position(blob[i]);
        }  
    }
    if(moving)
    {
        SetCenter();
    } else {
        centerChange = Vector2(0,0);
    }
    
    if (moveParent)
    {
        prnt -> set_position(prnt -> get_position()+centerChange);//
        set_position(get_position()-centerChange);
        
    } else {
        set_position(get_position()+centerChange);
        moveTo-=centerChange;
        
    }
     
    for (int i = 0; i < points; i++)
    {
        // decay changes
        changeCollision[i]*=collisionDecay;
        changeStiffness[i]*=stiffnessDecay;
        // Reset changes
        changeSpring[i]*=springDecay;
        changePressure[i]*=pressureDecay;
        changeMove[i]*=moveDecay;
    }
    // Update
    updateCollision();
    
    if (observerareaExists) updateObserverArea();
    updateSprite();
    update();
    
}
void Softbody2DPolygon::HardbodyPhysics(float delta)
{
    collisionHappened = false;
    // 1. calculate change in blob
    Vector2 change = Vector2(0,0);
    if (moving) // move to Target
    {
        if (center.distance_to(moveTo)<50)// if one point close enough stop
        {
            moving=false;
            centerChange = Vector2(0,0);
        }
        Vector2 direction = Vector2(0,0);
        
        direction = moveToRel(0);
        
        change = direction.normalized()*acceleration ; 
    }
    for (int i = 0; i < points; i++)
    {
        // ############ check collision
            
        if ((blobCollisions[i] -> get_slide_count()>0))
        {
            collisionHappened = true;
            change = -change*2;
            moving = false;
            break;
        }

        blobCollisions[i] -> move_and_slide(change);// so collision happens
    }
    // 2. apply change in blob
    for (int i = 0; i < points; i++)
    {
        blob[i] += change*delta;
    }

    update(); 
    // 3. calculate new center/centerChange
    if(moving)
    {
        SetCenter();
    } else {
        centerChange = Vector2(0,0);
    }
    // 4. deapply centerChange to blob
    for (int i = 0; i < points; i++)blob[i] -= centerChange; 
    // 5. apply centerChange to parent or self
    if (moveParent)
    {
        prnt -> set_position(prnt -> get_position()+centerChange);
        moveTo-=centerChange; // because local space moves moveTo has to change
    } else {
        set_position(get_position()+centerChange);
        moveTo-=centerChange; // because local space moves moveTo has to change
    }
    SetCenter();
    // 6. apply change in blob to all blob dependend bodies
    for (int i = 0; i < points; i++) blobCollisions[i] ->set_position(blob[i]);
}

void Softbody2DPolygon::enterHardbodyPhysics()
{
    useSoftbody=false;
    for (int i = 0; i < points; i++) 
    {
        // decay changes
        changeCollision[i]*=0.0;
        changeStiffness[i]*=0.0;
        // Reset changes
        changeSpring[i]*=0.;
        changePressure[i]*=0.;
        changeMove[i]*=0.;
    }
}
void Softbody2DPolygon::enterSoftbodyPhysics()
{
    useSoftbody=true;
    
}
void Softbody2DPolygon::move_and_slide(Vector2 velocity)
{
    moveTo = velocity;
    moving = true;
    //allowPhysics = true;
}
void Softbody2DPolygon::_draw()
{
    PoolVector2Array bakedPoints = curve -> get_baked_points();
    PoolVector2Array drawPoints = bakedPoints;
    if ( geometryObj->triangulate_polygon(bakedPoints).size()==0)
    {
        drawPoints = geometryObj->convex_hull_2d(bakedPoints);
    } 
    set_polygon(drawPoints);
} 

