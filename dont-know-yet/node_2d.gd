extends Node2D



var parSize : int = 1
var isLeftMouseButtonHeld = false
var isRightMouseButtonHeld = false

@export var numberOfParticle : int = 91
@export var particalSpacing : float = 1

@export var collisionDamping : float = 0.05

@export var boundsSize : Vector2  = Vector2(1000, 500)

@export var smoothingRadius : float = 30

@export var targetDensity : float 
@export var pressureMultilier : float 

@export var gradient : Gradient 
@export var viscosityStrength : float 

const mass : float = 1
var densities : Array[float]

var points : Array[Vector2]
var radius : float
var spacialLookup : Array[Array]
var startIndices : Array[int]
var cellOffsets : Array[Vector2i] = [	Vector2i(0, 0), Vector2i(0, 1), Vector2i(0, -1), 
										Vector2i(1, 0), Vector2i(1, 1), Vector2i(1, -1), 
										Vector2i(-1, 0), Vector2i(-1, 1), Vector2i(-1, -1)]

var neighborPar : Callable = Callable(self, "foreachPointWithinRadius")

@export var gravity : float 
var parPositions : Array[Vector2] = []
var parVelocities : Array[Vector2] = []
var predictedPosition : Array[Vector2] = []

func _draw() -> void:
	draw_line(-(boundsSize / 2) , Vector2((boundsSize / 2).x, -(boundsSize / 2).y) , Color.GREEN)
	draw_line(-(boundsSize / 2) , Vector2(-(boundsSize / 2).x, (boundsSize / 2).y) , Color.GREEN)
	draw_line((boundsSize / 2) , Vector2((boundsSize / 2).x, -(boundsSize / 2).y) , Color.GREEN)
	draw_line((boundsSize / 2) , Vector2(-(boundsSize / 2).x, (boundsSize / 2).y) , Color.GREEN)
	for n in parPositions.size() :
		draw_circle(parPositions[n], parSize , gradient.sample(inverse_lerp(0, 150, parVelocities[n].length_squared())))


func _ready() -> void:
	parPositions.resize(numberOfParticle)
	parVelocities.resize(numberOfParticle)
	densities.resize(numberOfParticle)
	spacialLookup.resize(numberOfParticle)
	startIndices.resize(numberOfParticle * 2)
	predictedPosition.resize(numberOfParticle)
	
	
	var particlesPerRow : int = roundi(sqrt(numberOfParticle))
	var particlesPerCol : int  = (numberOfParticle - 1) / particlesPerRow + 1
	var spacing = parSize * 2 + particalSpacing
	
	for i in numberOfParticle : 
		var x : float = (i % particlesPerRow - particlesPerRow / 2 + 0.5) * spacing
		var y : float = (i / particlesPerRow - particlesPerCol / 2 + 0.5) * spacing
		parPositions[i] = Vector2(x, y)
	
	


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	for i in numberOfParticle :
		parVelocities[i] += Vector2.DOWN * gravity * delta
		predictedPosition[i] = parPositions[i] + parVelocities[i] * 1 / 120.0
	
	updateSpacialLookup(predictedPosition, smoothingRadius)
	
	for i in numberOfParticle :
		densities[i] = caculateDensity(predictedPosition[i])
	
	for i in numberOfParticle :
		var pressureForce : Vector2 = calulatePressureForce(i)
		var pressureAcceleration = pressureForce / densities[i]
		parVelocities[i] += pressureAcceleration * delta
		parVelocities[i] += caculateViscosityForce(i)
		if isLeftMouseButtonHeld :
			parVelocities[i] += interactionForce(get_global_mouse_position(), 12, -30, i)
		elif isRightMouseButtonHeld :
			parVelocities[i] += interactionForce(get_global_mouse_position(), 12, 5, i)
	
	for i in numberOfParticle :
		parPositions[i] += parVelocities[i] * delta
		ResolveCollisions(parPositions[i], parVelocities[i], i)
	
	queue_redraw()


func ResolveCollisions(parPosition : Vector2 , parVelocity : Vector2, parIndex : int) -> void :
	var halfBoundsSize = (boundsSize / 2) - Vector2.ONE * parSize
	
	if (abs(parPosition.x) > halfBoundsSize.x) :
		parPosition.x = halfBoundsSize.x * sign(parPosition.x)
		parVelocity.x *= -1 * collisionDamping
	if (abs(parPosition.y) > halfBoundsSize.y) :
		parPosition.y = halfBoundsSize.y * sign(parPosition.y)
		parVelocity.y *= -1 * collisionDamping
	
	parPositions[parIndex] = parPosition
	parVelocities[parIndex] = parVelocity

func smoothingKernel (paramRadius : float , dst : float) -> float :
	
	if ( dst >= paramRadius ) : return 0
	
	var volume : float = (PI * pow(paramRadius, 4)) / 6
	return (paramRadius - dst) * (paramRadius - dst) / volume

func smoothingKernelDerivative ( dst : float , paramRadius : float ) -> float :
	if ( dst >= paramRadius ) :
		return 0
	
	var derivativeScale : float = 12 / (pow(paramRadius, 4) * PI)
	return (dst - paramRadius) * derivativeScale


func caculateDensity( samplePoint : Vector2 ) -> float :
	var density : float = 0
	var nearParPos : Array[Vector2] = []
	for i in neighborPar.call( samplePoint ) :
		nearParPos.append(predictedPosition[i])
	for i in nearParPos :
		var dst = (i - samplePoint).length()
		var influence = smoothingKernel(smoothingRadius, dst)
		density += mass * influence
	return density

func calulatePressureForce ( particalIndex : int  ) -> Vector2 :
	var pressureForce : Vector2 = Vector2.ZERO
	var nearParPos : Array[int] = neighborPar.call( predictedPosition[particalIndex] )
	
	for otherParticalIndex in nearParPos :
		if ( particalIndex == otherParticalIndex ) :
			continue
		var offset : Vector2 = predictedPosition[otherParticalIndex] - predictedPosition[particalIndex]
		var dst : float = offset.length()
		var dir : Vector2 =  Vector2.RIGHT.rotated(deg_to_rad(randi_range(0, 360))) if dst == 0 else offset / dst
		var slope = smoothingKernelDerivative(dst, smoothingRadius);
		var density : float = densities[otherParticalIndex]
		var sharedPressure = caculateSharedPressure(density, densities[particalIndex])
		pressureForce += -sharedPressure * dir * slope * mass / density
	return pressureForce


func caculateSharedPressure(densityA : float, densityB : float) -> float :
	var pressureA : float = convertDensityToPressure(densityA)
	var pressureB : float = convertDensityToPressure(densityB)
	return (pressureA + pressureB) / 2


func convertDensityToPressure(density : float) -> float :
	var densityError : float = density - targetDensity
	var pressure : float = densityError * pressureMultilier
	return pressure


func updateSpacialLookup(paramPoints : Array[Vector2], paramRadius : float) -> void :
	self.points = paramPoints
	self.radius = paramRadius
	
	for i in points.size() :
		var temp : Vector2i = positionToCellCoord(points[i], radius)
		var cellKey : int = getKeyFromHash(hashCell(temp.x, temp.y))
		spacialLookup[i] = [i, cellKey]
		startIndices[i] = 9223372036854775807
		startIndices[-i] = 9223372036854775807
	
	spacialLookup.sort_custom(sort_ascending)
	
	for i in points.size() :
		var key : int = spacialLookup[i][1]
		var keyPrev : int = 9223372036854775807 if i == 0 else spacialLookup[i -1][1]
		if (key != keyPrev) :
			startIndices[key] = i




func sort_ascending(a : Array, b : Array):
	if a[1] < b[1]:
		return true
	return false



func positionToCellCoord(point : Vector2, paramRadius : float) -> Vector2i :
	var cellX : int = point.x / paramRadius
	var cellY : int = point.y / paramRadius
	return Vector2i(cellX, cellY)

func hashCell(cellX : int, cellY : int) -> int :
	var a : int = cellX * 15823
	var b : int = cellY * 9737333
	return a + b

func getKeyFromHash(paramHash : int) -> int :
	return paramHash % spacialLookup.size()


func foreachPointWithinRadius(samplePoint : Vector2) -> Array[int] :
	var center : Vector2 = positionToCellCoord(samplePoint, radius)
	var sqrRadius = radius * radius
	var nearParIndex : Array[int] = []
	
	for offset in cellOffsets :
		var key : int = getKeyFromHash(hashCell(center.x + offset.x, center.y + offset.y))
		var cellStartIndex : int = startIndices[key]
		for i in range(cellStartIndex, spacialLookup.size()) :
			if ( spacialLookup[i][1] != key) :
				break
			var particleIndex : int = spacialLookup[i][0]
			var sqrDst : float = (points[particleIndex] - samplePoint).length_squared()
			
			if (sqrDst <= sqrRadius) :
				nearParIndex.append(particleIndex)
	
	return nearParIndex


func interactionForce(inputPos : Vector2, paramRadius : float, strength : float, particleIndex : int) -> Vector2 :
	var result : Vector2 = Vector2.ZERO
	var offset : Vector2 = inputPos - parPositions[particleIndex]
	var sqrDist = offset.dot(offset)
	
	if ( sqrDist < paramRadius * paramRadius) :
		var dist = sqrt(sqrDist)
		var dirToInputPoint = Vector2.ZERO if is_zero_approx(dist) else offset / dist
		var centerT : float = 1 - dist / paramRadius
		result += (dirToInputPoint * strength - parVelocities[particleIndex]) * centerT
	
	return result 


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton && event.is_pressed():
		match event.button_index :
			MOUSE_BUTTON_LEFT :
				isLeftMouseButtonHeld = true if isLeftMouseButtonHeld == false else false
			MOUSE_BUTTON_RIGHT  :
				isRightMouseButtonHeld = true if isRightMouseButtonHeld == false else false
	elif event is InputEventMouseButton && event.is_released():
		match event.button_index :
			MOUSE_BUTTON_LEFT :
				isLeftMouseButtonHeld = false if isLeftMouseButtonHeld == true else true
			MOUSE_BUTTON_RIGHT  :
				isRightMouseButtonHeld = false if isRightMouseButtonHeld == true else true



func caculateViscosityForce(particleIndex) -> Vector2 :
	var viscosityForce = Vector2.ZERO
	var parPosition = parPositions[particleIndex]
	
	for otherIndex in neighborPar.call( parPosition ) :
		var dist : float = (parPosition - parPositions[otherIndex]).length()
		var influence = vicoscitySmoothingKernel(dist, smoothingRadius)
		viscosityForce += (parVelocities[otherIndex] - parVelocities[particleIndex]) * influence
	
	return viscosityForce * viscosityStrength


func vicoscitySmoothingKernel(dist : float, paramRadius : float) -> float :
	if (dist < paramRadius) : return 0 
	var volume : float = PI * pow(paramRadius, 8) / 4
	var value : float = max(0, paramRadius * paramRadius - dist * dist)
	return value * value * value / volume
