package main

import (
	"log"
	"math"

	"github.com/bytearena/bytearena/common/utils/vector"
)

func main() {
	log.Println("yooo")

	car := &Car{

		// State
		Heading:    0.0,
		Position:   vector.MakeNullVector2(),
		Velocity:   vector.MakeNullVector2(),
		Velocity_c: vector.MakeNullVector2(),
		Accel:      vector.MakeNullVector2(),
		Accel_c:    vector.MakeNullVector2(),
		AbsVel:     0.0,
		YawRate:    0.0,
		Steer:      0.0,
		SteerAngle: 0.0,

		// Derived state
		Inertia:              0.0,
		WheelBase:            0.0,
		AxleWeightRatioFront: 0.0,
		AxleWeightRatioRear:  0.0,

		// Controls
		Left:     0.0,
		Right:    0.0,
		Throttle: 0.0,
		Brake:    0.0,
		Ebrake:   0.0,

		// Constants
		Gravity:              9.81,
		Mass:                 1200.0,
		InertiaScale:         1.0,
		HalfWidth:            0.8,
		CgToFront:            2.0,
		CgToRear:             2.0,
		CgToFrontAxle:        1.25,
		CgToRearAxle:         1.25,
		CgHeight:             0.55,
		WheelRadius:          0.3,
		WheelWidth:           0.2,
		TireGrip:             2.0,
		LockGrip:             0.7,
		EngineForce:          8000.0,
		BrakeForce:           12000.0,
		EBrakeForce:          12000.0 / 2.5,
		WeightTransfer:       0.2,
		MaxSteer:             0.6,
		CornerStiffnessFront: 5.0,
		CornerStiffnessRear:  5.2,
		AirResist:            2.5,
		RollResist:           8.0,
	}

	car.Setup()
	car.Throttle = 1.0
	for k := 0.0; k < 60.0; k++ {
		car.Update((1000.0 / 60.0) * k)
		log.Println(car.Position)
	}
}

type Car struct {

	//  Car state variables
	Heading    float64        // angle car is pointed at (radians)
	Position   vector.Vector2 // metres, world coords
	Velocity   vector.Vector2 // m/s in world coords
	Velocity_c vector.Vector2 // m/s in local car coords (x is forward y is sideways)
	Accel      vector.Vector2 // acceleration in world coords
	Accel_c    vector.Vector2 // acceleration in local car coords
	AbsVel     float64        // absolute velocity in m/s
	YawRate    float64        // angular velocity in radians
	Steer      float64        // amount of steering input (-1.0..1.0)
	SteerAngle float64        // actual front wheel steer angle (-maxSteer..maxSteer)

	//  State of inputs
	Left     float64
	Right    float64
	Throttle float64
	Brake    float64
	Ebrake   float64

	//  Other static values to be computed from config
	Inertia              float64 // will be = mass
	WheelBase            float64 // set from axle to CG lengths
	AxleWeightRatioFront float64 // % car weight on the front axle
	AxleWeightRatioRear  float64 // % car weight on the rear axle

	//  Setup car configuration
	Gravity              float64 // m/s^2
	Mass                 float64 // kg
	InertiaScale         float64 // Multiply by mass for inertia
	HalfWidth            float64 // Centre to side of chassis (metres)
	CgToFront            float64 // Centre of gravity to front of chassis (metres)
	CgToRear             float64 // Centre of gravity to rear of chassis
	CgToFrontAxle        float64 // Centre gravity to front axle
	CgToRearAxle         float64 // Centre gravity to rear axle
	CgHeight             float64 // Centre gravity height
	WheelRadius          float64 // Includes tire (also represents height of axle)
	WheelWidth           float64 // Used for render only
	TireGrip             float64 // How much grip tires have
	LockGrip             float64 // % of grip available when wheel is locked
	EngineForce          float64
	BrakeForce           float64
	EBrakeForce          float64
	WeightTransfer       float64 // How much weight is transferred during acceleration/braking
	MaxSteer             float64 // Maximum steering angle in radians
	CornerStiffnessFront float64
	CornerStiffnessRear  float64
	AirResist            float64 // air resistance (* vel)
	RollResist           float64 // rolling resistance force (* vel)
}

func (car *Car) Setup() {
	car.Inertia = car.Mass * car.InertiaScale
	car.WheelBase = car.CgToFrontAxle + car.CgToRearAxle
	car.AxleWeightRatioFront = car.CgToRearAxle / car.WheelBase // % car weight on the front axle
	car.AxleWeightRatioRear = car.CgToFrontAxle / car.WheelBase // % car weight on the rear axle
}

func (car *Car) Update(dtms float64) {
	car.Steer = car.Left - car.Right
	car.SteerAngle = car.Steer * car.MaxSteer
	car.doPhysics(dtms)
}

func sign(n float64) int {
	if n < 0 {
		return -1
	}

	return 1
}

func clamp(n float64, min float64, max float64) float64 {
	return math.Min(math.Max(n, min), max)
}

func (car *Car) doPhysics(dtms float64) {
	dt := dtms / 1000.0 // delta T in seconds

	// Pre-calc heading vector
	sn := math.Sin(car.Heading)
	cs := math.Cos(car.Heading)

	// Get velocity in local car coordinates
	car.Velocity_c = car.Velocity_c.
		SetX(cs*car.Velocity.GetX() + sn*car.Velocity.GetY()).
		SetY(cs*car.Velocity.GetY() - sn*car.Velocity.GetX())

	// Weight on axles based on centre of gravity and weight shift due to forward/reverse acceleration
	axleWeightFront := car.Mass * (car.AxleWeightRatioFront*car.Gravity - car.WeightTransfer*car.Accel_c.GetX()*car.CgHeight/car.WheelBase)
	axleWeightRear := car.Mass * (car.AxleWeightRatioRear*car.Gravity + car.WeightTransfer*car.Accel_c.GetX()*car.CgHeight/car.WheelBase)

	// Resulting velocity of the wheels as result of the yaw rate of the car body.
	// v = yawrate * r where r is distance from axle to CG and yawRate (angular velocity) in rad/s.
	yawSpeedFront := car.CgToFrontAxle * car.YawRate
	yawSpeedRear := -car.CgToRearAxle * car.YawRate

	// Calculate slip angles for front and rear wheels (a.k.a. alpha)
	slipAngleFront := math.Atan2(car.Velocity_c.GetY()+yawSpeedFront, math.Abs(car.Velocity_c.GetX())-float64(sign(car.Velocity_c.GetX()))*car.SteerAngle)
	slipAngleRear := math.Atan2(car.Velocity_c.GetY()+yawSpeedRear, math.Abs(car.Velocity_c.GetX()))

	tireGripFront := car.TireGrip
	tireGripRear := car.TireGrip * (1.0 - car.Ebrake*(1.0-car.LockGrip)) // reduce rear grip when ebrake is on

	frictionForceFront_cy := clamp(-car.CornerStiffnessFront*slipAngleFront, -tireGripFront, tireGripFront) * axleWeightFront
	frictionForceRear_cy := clamp(-car.CornerStiffnessRear*slipAngleRear, -tireGripRear, tireGripRear) * axleWeightRear

	//  Get amount of brake/throttle from our inputs
	brake := math.Min(car.Brake*car.BrakeForce+car.Ebrake*car.EBrakeForce, car.BrakeForce)
	throttle := car.Throttle * car.EngineForce

	//  Resulting force in local car coordinates.
	//  This is implemented as a RWD car only.
	tractionForce_cx := throttle - brake*float64(sign(car.Velocity_c.GetX()))
	tractionForce_cy := 0.0

	dragForce_cx := -car.RollResist*car.Velocity_c.GetX() - car.AirResist*car.Velocity_c.GetX()*math.Abs(car.Velocity_c.GetX())
	dragForce_cy := -car.RollResist*car.Velocity_c.GetY() - car.AirResist*car.Velocity_c.GetY()*math.Abs(car.Velocity_c.GetY())

	// total force in car coordinates
	totalForce_cx := dragForce_cx + tractionForce_cx
	totalForce_cy := dragForce_cy + tractionForce_cy + math.Cos(car.SteerAngle)*frictionForceFront_cy + frictionForceRear_cy

	// acceleration along car axes
	car.Accel_c = car.Accel_c.
		SetX(totalForce_cx / car.Mass). // forward/reverse accel
		SetY(totalForce_cy / car.Mass)  // sideways accel

	// acceleration in world coordinates
	car.Accel = car.Accel.
		SetX(cs*car.Accel_c.GetX() - sn*car.Accel_c.GetY()).
		SetY(sn*car.Accel_c.GetX() + cs*car.Accel_c.GetY())

	// update velocity
	car.Velocity = car.Velocity.
		SetX(car.Velocity.GetX() + car.Accel.GetX()*dt).
		SetY(car.Velocity.GetY() + car.Accel.GetY()*dt)

	car.AbsVel = car.Velocity.Mag()

	// calculate rotational forces
	angularTorque := (frictionForceFront_cy+tractionForce_cy)*car.CgToFrontAxle - frictionForceRear_cy*car.CgToRearAxle

	//  Sim gets unstable at very slow speeds, so just stop the car
	if math.Abs(car.AbsVel) < 0.5 && throttle == 0 {
		car.Velocity = vector.MakeVector2(0, 0)
		car.AbsVel = 0
		angularTorque = 0
		car.YawRate = 0
	}

	angularAccel := angularTorque / car.Inertia

	car.YawRate += angularAccel * dt
	car.Heading += car.YawRate * dt

	//  finally we can update position
	car.Position = car.Position.
		SetX(car.Velocity.GetX() * dt).
		SetY(car.Velocity.GetY() * dt)

	//  Display some data
	log.Println("speed", car.Velocity_c.GetX()*3600/1000) // km/h
	log.Println("accleration", car.Accel_c.GetX())
	log.Println("yawRate", car.YawRate)
	log.Println("weightFront", axleWeightFront)
	log.Println("weightRear", axleWeightRear)
	log.Println("slipAngleFront", slipAngleFront)
	log.Println("slipAngleRear", slipAngleRear)
	log.Println("frictionFront", frictionForceFront_cy)
	log.Println("frictionRear", frictionForceRear_cy)
}
