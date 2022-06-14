extends Node2D

var controlled_Node
var controlled_Node1
var controlled_NodeClick

func _ready():
	"""
	$Control/acceleration.connect("value_changed",self,"_on_acc_value_changed")
	$Control/springFactor.connect("value_changed",self,"_on_springFactor_value_changed")
	$Control/pressure.connect("value_changed",self,"_on_pressure_value_changed")
	$Control/collisionFactor.connect("value_changed",self,"_on_col_value_changed")
	$Control/Stiffness.connect("value_changed",self,"_on_gravity_value_changed")
	$Control/area.connect("value_changed",self,"_on_area_value_changed")
	
	$Control/MoveRelAbs.connect("pressed",self,"_on_MoveRel_pressed")
	$Control/useSoftbody.connect("pressed",self,"_on_useSoftbody_pressed")
	
	$Control/moveDecay.connect("value_changed",self,"_on_moveDecay_value_changed")
	$Control/springDecay.connect("value_changed",self,"_on_springDecay_value_changed")
	$Control/pressureDecay.connect("value_changed",self,"_on_pressureDecay_value_changed")
	$Control/collisionDecay.connect("value_changed",self,"_on_collisionDecay_value_changed")
	$Control/stiffnessDecay.connect("value_changed",self,"_on_stiffnessDecay_value_changed")
	"""
	controlled_Node = $Polygon2DCircle
	
	controlled_NodeClick = $Polygon2D
	
	controlled_Node1 = $Polygon2DCircle
	
"""
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

func _on_moveDecay_value_changed(x):
	controlled_Node.moveDecay = x
func _on_springDecay_value_changed(x):
	controlled_Node.springDecay = x
func _on_pressureDecay_value_changed(x):
	controlled_Node.pressureDecay = x
func _on_collisionDecay_value_changed(x):
	controlled_Node.collisionDecay = x
func _on_stiffnessDecay_value_changed(x):
	controlled_Node.stiffnessDecay = x

func _on_area_value_changed(x):
	controlled_Node.area = controlled_Node.areaInitial * x

func _on_MoveRel_pressed():
	controlled_Node.moveRel = $Control/MoveRelAbs.pressed;

func _on_useSoftbody_pressed():
	controlled_Node.useSoftbody = $Control/MoveRelAbs.pressed;

func _unhandled_input(event):
	# controlled_Node Click controlled
	if event.is_action_pressed("leftClick"):
		controlled_NodeClick.set("allowPhysics",true);
		controlled_NodeClick.move_and_slide(controlled_NodeClick.get_local_mouse_position())
"""
	
func _physics_process(delta):
	#controlled_Node1 wasd controlled
	if not controlled_Node1: return
	var moveVec = Vector2.DOWN * 100# gravity

	if Input.is_action_pressed("ui_left"):
		moveVec+= Vector2.LEFT * 100
	if Input.is_action_pressed("ui_right"):
		moveVec+= Vector2.RIGHT * 100
	if Input.is_action_pressed("ui_select"):
		moveVec+= Vector2.UP * 200
	moveVec+=controlled_Node1.center;
	controlled_Node1.move_and_slide(moveVec)

func _draw():
	#draw_circle(controlled_Node.moveTo,5,Color(0,0,1))
	pass
