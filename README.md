# GodotSoftBody2D
Derives from https://github.com/Lynext/GodotSoftBody2D.

Testscene.tscn is the initial testscene from https://github.com/Lynext/GodotSoftBody2D, but with a version of the slime translated into c#.
Testscene2.tscn is there for testing the current softbodies.

I currently made a circle-softbody and a free one, which you can draw via a Polygon2D node. The lines of the Polygon2D are made curvy via code by forming them into bezier-curve with one extra point in the middle with the normal added to it.

The Softbody-Model is closest to a pressure-spring model.
It has a spring force, which pulls adjacent nodes together if too far apart and pushes them apart if too close together.
It has a pressure force, which pushes nodes along their normals. It pushes out if the area is too small and pushes in if the area is too big.
It has a collision force, which pushes against the collision. Was supposed to introduce bouncieness, but currently only intensifies collision.
It has a "stiffness" force, which pulls nodes towards their original position relativ to the center. It acts like pressure force, but also forces the softbody back into its original shape.
And there is a Movement force with two modi. For movement you set a Vector2 moveTo. If moveRel=True every node moves along the vector between center and moveTo. If moveRel=False every node moves directly towards moveTo.

The softbodies themself are Polygon2Ds with Kineticbody2Ds with round CollisionShapes attached to every node.
