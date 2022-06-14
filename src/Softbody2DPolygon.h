#ifndef Softbody2DPolygon_H
#define Softbody2DPolygon_H

//## Godot includes
#include <godot.hpp>
#include <polygon2d.hpp>
#include <Vector2.hpp>
#include <Curve2D.hpp>
#include <KinematicCollision2D.hpp>
#include <KinematicBody2D.hpp>
#include <Node2D.hpp>
#include <CollisionShape2D.hpp>
#include <CollisionPolygon2D.hpp>
#include <Area2D.hpp>
#include <CircleShape2D.hpp>
#include <Ref.hpp>
#include <Geometry.hpp>
//## Basic includes
#include <vector>
#include <math.h>
#include <string>

namespace godot
{
    class Softbody2DPolygon : public Polygon2D
    {
        GODOT_CLASS(Softbody2DPolygon,Polygon2D)
        private:

        public:
            // ########Vars
            int points = 2;
            float area;
            float areaInitial;
            float circumfrence;
            std::vector<float> lengths;
            std::vector<float> lengthsInitial;
            std::vector<Vector2> blob;
            std::vector<Vector2> blobInitial2Center;
            std::vector<Vector2> changeSpring;
            std::vector<Vector2> changePressure;
            std::vector<Vector2> changeCollision;
            std::vector<Vector2> changeMove;
            std::vector<Vector2> changeStiffness;
            std::vector<KinematicBody2D*> blobCollisions;
            std::vector<CollisionShape2D*> blobColShapes;
            std::vector<Vector2> normals;
            Vector2 moveTo = Vector2(0,0);
            bool moving = false;

            float acceleration;
            float springFactor;
            float pressureFactor;
            float stiffnessFactor;
            float collisionFactor;

            float acceleration_ = 80.0;
            float springFactor_ = 20.0;
            float pressureFactor_ = 20.0;
            float stiffnessFactor_ = 10.0;
            float collisionFactor_ = 50.0;

            float lengthSet;
            float lengthSet_ = 40.0;

            float moveDecay ;
            float springDecay ;
            float pressureDecay ;
            float stiffnessDecay ;
            float collisionDecay ;
            
            float moveDecay_ = 0.0;
            float springDecay_ = 0.0;
            float pressureDecay_ = 0.0;
            float stiffnessDecay_ = 0.8;
            float collisionDecay_ = 0.8;

            float fps = 30;
            float fps_ = 30;
            
            float timeHelper = 0;

            float collisionRadius;
            float splineLength = 12;
            Curve2D* curve = Curve2D::_new();
            PoolVector2Array polygonInitial;
            bool moveRel = false;
            Vector2 center = Vector2(0,0);
            Vector2 centerChange;
            bool allowPhysics = false;
            std::vector<KinematicCollision2D> Colliders;
            Node2D* prnt;// this is parent
            CollisionPolygon2D* AreaShape;
            Area2D* ObserverArea;
            float ObserverAreaRadius = 5;

            int body_count = 0;
            bool useSoftbody = true;
            bool moveParent = false;
            bool createObserverarea = false;
            bool observerareaExists = false;
            bool collisionHappened = true;

            bool use_acceleration = true;
            bool use_springFactor = true;
            bool use_pressureFactor = true;
            bool use_stiffnessFactor = true;
            bool use_collisionFactor = true;

            Geometry* geometryObj = Geometry::get_singleton();

            KinematicBody2D* hardCollisionBody ;
            CollisionPolygon2D* hardCollisionPolygon ;
            // ########methods
            static void _register_methods();
            Softbody2DPolygon();
            ~Softbody2DPolygon();   
            void _init();   
            void _ready();
            void _physics_process(float delta);
            void _draw();
            void resetBlob();
            Vector2 calculateNormal(int i);
            Vector2 getVectorByAngle(float length, float angle);
            float getCurArea();
            void SetCenter();
            void initBlobCollisions();
            void updateSprite();
            Vector2 getSpline(int i);
            Vector2 getPoint(int i);
            Vector2 moveToRel(int i);
            Vector2 moveToAbs(int i);
            void updateCollision();
            void adjustPolygon();
            void initObserverArea();
            void on_body_entered(Variant body);
            void on_body_exited(Variant body);
            void SoftbodyPhysics(float delta);
            void HardbodyPhysics(float delta);
            void updateObserverArea();
            void initHardCollision();
            void updateHardCollision();
            void enterSoftbodyPhysics();
            void enterHardbodyPhysics();
            void move_and_slide(Vector2 velocity);
            
            
            
    };
    
} 


#endif