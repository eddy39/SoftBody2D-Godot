extends Node2D


func _ready():
	$CanvasLayer/HSlider.connect("value_changed",$SlimeCs,"_on_HSlider_value_changed")
	$CanvasLayer/HSlider2.connect("value_changed",$SlimeCs,"_on_HSlider2_value_changed")
	$CanvasLayer/HSlider3.connect("value_changed",$SlimeCs,"_on_HSlider3_value_changed")
	$CanvasLayer/HSlider4.connect("value_changed",$SlimeCs,"_on_HSlider4_value_changed")
	
