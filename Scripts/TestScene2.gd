extends Node2D

var controlled_Node
func _ready():
	$Control/speed.connect("value_changed",self,"_on_speed_value_changed")
	$Control/acceleration.connect("value_changed",self,"_on_acc_value_changed")
	$Control/springFactor.connect("value_changed",self,"_on_springFactor_value_changed")
	$Control/pressure.connect("value_changed",self,"_on_pressure_value_changed")
	$Control/collisionFactor.connect("value_changed",self,"_on_col_value_changed")
	$Control/Stiffness.connect("value_changed",self,"_on_gravity_value_changed")
	$Control/points.connect("value_changed",self,"_on_points_value_changed")
	$Control/area.connect("value_changed",self,"_on_area_value_changed")
	$Control/MoveRelAbs.connect("pressed",self,"_on_MoveRel_pressed")
	controlled_Node = $SoftBody2DCircle
	
func _on_speed_value_changed(x):
	controlled_Node.speed = x
func _on_acc_value_changed(x):
	controlled_Node.acceleration = x
func _on_springFactor_value_changed(x):
	controlled_Node.springFactor = x
func _on_pressure_value_changed(x):
	controlled_Node.pressureFactor = x
func _on_col_value_changed(x):
	controlled_Node.collisionFactor = x
func _on_gravity_value_changed(x):
	controlled_Node.stiffnessFactor = x
func _on_area_value_changed(x):
	controlled_Node.area = controlled_Node.areaInitial * x
func _on_points_value_changed(x):
	controlled_Node.points = int(x)
	controlled_Node.resetBlob()
func _on_MoveRel_pressed():
	controlled_Node.moveRel = $Control/MoveRelAbs.pressed;
func _unhandled_input(event):
	if event.is_action_pressed("leftClick"):
		controlled_Node.moving = true;
		controlled_Node.moveTo = controlled_Node.get_local_mouse_position();
	pass
func _draw():
	draw_circle(controlled_Node.moveTo,5,Color(0,0,1))
