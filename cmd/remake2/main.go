package main

import (
	"fmt"
	"log"
	"math"
	"os"
	"text/tabwriter"
	"time"

	"github.com/bytearena/bytearena/common/utils/vector"
)

type Car struct {
	//  Car state variables
	heading    float64        // car.heading = opts.heading || 0.0;  // angle car is pointed at (radians)
	position   vector.Vector2 // car.position = new Vec2(opts.x, opts.y);  // metres in world coords
	velocity   vector.Vector2 // car.velocity = new Vec2();  // m/s in world coords
	velocity_c vector.Vector2 // car.velocity_c = new Vec2();  // m/s in local car coords (x is forward y is sideways)
	accel      vector.Vector2 // car.accel = new Vec2();  // acceleration in world coords
	accel_c    vector.Vector2 // car.accel_c = new Vec2();   // accleration in local car coords
	absVel     float64        // car.absVel = 0.0;  // absolute velocity m/s
	yawRate    float64        // car.yawRate = 0.0;   // angular velocity in radians

	//  Other static values to be computed from config
	inertia              float64 // car.inertia = 0.0;  // will be = mass
	wheelBase            float64 // car.wheelBase = 0.0;  // set from axle to CG lengths
	axleWeightRatioFront float64 // car.axleWeightRatioFront = 0.0;  // % car weight on the front axle
	axleWeightRatioRear  float64 // car.axleWeightRatioRear = 0.0;  // % car weight on the rear axle

	//  Defaults approximate a lightweight sports-sedan.
	gravity              float64 // car.gravity = opts.gravity || 9.81;  // m/s^2
	mass                 float64 // car.mass = opts.mass || 1200.0;  // kg
	inertiaScale         float64 // car.inertiaScale = opts.inertiaScale || 1.0;  // Multiply by mass for inertia
	halfWidth            float64 // car.halfWidth = opts.halfWidth || 0.8; // Centre to side of chassis (metres)
	cgToFront            float64 // car.cgToFront = opts.cgToFront || 2.0; // Centre of gravity to front of chassis (metres)
	cgToRear             float64 // car.cgToRear = opts.cgToRear || 2.0;   // Centre of gravity to rear of chassis
	cgToFrontAxle        float64 // car.cgToFrontAxle = opts.cgToFrontAxle || 1.25;  // Centre gravity to front axle
	cgToRearAxle         float64 // car.cgToRearAxle = opts.cgToRearAxle || 1.25;  // Centre gravity to rear axle
	cgHeight             float64 // car.cgHeight = opts.cgHeight || 0.55;  // Centre gravity height
	wheelRadius          float64 // car.wheelRadius = opts.wheelRadius || 0.3;  // Includes tire (also represents height of axle)
	tireGrip             float64 // car.tireGrip = opts.tireGrip || 2.0;  // How much grip tires have
	lockGrip             float64 // car.lockGrip = (typeof opts.lockGrip === 'number') ? GMath.clamp(opts.lockGrip, 0.01, 1.0) : 0.7;  // % of grip available when wheel is locked
	engineForce          float64 // car.engineForce = opts.engineForce || 8000.0;
	brakeForce           float64 // car.brakeForce = opts.brakeForce || 12000.0;
	eBrakeForce          float64 // car.eBrakeForce = opts.eBrakeForce || car.brakeForce / 2.5;
	weightTransfer       float64 // car.weightTransfer = (typeof opts.weightTransfer === 'number') ? opts.weightTransfer : 0.2;  // How much weight is transferred during acceleration/braking
	maxSteer             float64 // car.maxSteer = opts.maxSteer || 0.6;  // Maximum steering angle in radians
	cornerStiffnessFront float64 // car.cornerStiffnessFront = opts.cornerStiffnessFront || 5.0;
	cornerStiffnessRear  float64 // car.cornerStiffnessRear = opts.cornerStiffnessRear || 5.2;
	airResist            float64 // car.airResist = (typeof opts.airResist === 'number') ? opts.airResist : 2.5;	// air resistance (* vel)
	rollResist           float64 // car.rollResist = (typeof opts.rollResist === 'number') ? opts.rollResist : 8.0;	// rolling resistance force (* vel)

	// Inputs
	inputEbrake   float64
	inputBrake    float64
	inputThrottle float64 // amount of throttle (0.0..1.0)
	inputSteer    float64 // amount of steering input (-1.0..1.0)
}

func (car *Car) init() {
	car.inertia = car.mass * car.inertiaScale
	car.wheelBase = car.cgToFrontAxle + car.cgToRearAxle
	car.axleWeightRatioFront = car.cgToRearAxle / car.wheelBase // % car weight on the front axle
	car.axleWeightRatioRear = car.cgToFrontAxle / car.wheelBase // % car weight on the rear axle
}

func main() {
	vehicle := &Car{
		//  Car state variables
		heading:    0.0,                      // angle car is pointed at (radians)
		position:   vector.MakeNullVector2(), // metres in world coords
		velocity:   vector.MakeNullVector2(), // m/s in world coords
		velocity_c: vector.MakeNullVector2(), // m/s in local car coords (x is forward y is sideways)
		accel:      vector.MakeNullVector2(), // acceleration in world coords
		accel_c:    vector.MakeNullVector2(), // accleration in local car coords
		absVel:     0.0,                      // absolute velocity m/s
		yawRate:    0.0,                      // angular velocity in radians

		//  Other static values to be computed from config
		inertia:              0.0, // will be = mass
		wheelBase:            0.0, // set from axle to CG lengths
		axleWeightRatioFront: 0.0, // % car weight on the front axle
		axleWeightRatioRear:  0.0, // % car weight on the rear axle

		//  Defaults approximate a lightweight sports-sedan.
		gravity:              9.80665, // m/s^2
		mass:                 1200.0,  // kg
		inertiaScale:         1.0,     // Multiply by mass for inertia
		halfWidth:            0.8,     // Centre to side of chassis (metres)
		cgToFront:            2.0,     // Centre of gravity to front of chassis (metres)
		cgToRear:             2.0,     // Centre of gravity to rear of chassis
		cgToFrontAxle:        1.25,    // Centre gravity to front axle
		cgToRearAxle:         1.25,    // Centre gravity to rear axle
		cgHeight:             0.55,    // Centre gravity height
		wheelRadius:          0.3,     // Includes tire (also represents height of axle)
		tireGrip:             2.0,     // How much grip tires have
		lockGrip:             0.7,     // % of grip available when wheel is locked
		engineForce:          8000.0,
		brakeForce:           12000.0,
		eBrakeForce:          4800.0,
		weightTransfer:       0.2, // How much weight is transferred during acceleration/braking
		maxSteer:             0.6, // Maximum steering angle in radians
		cornerStiffnessFront: 5.0,
		cornerStiffnessRear:  5.2,
		airResist:            2.5, // air resistance (* vel)
		rollResist:           8.0, // rolling resistance force (* vel)

		inputEbrake:   0.0,
		inputBrake:    0.0,
		inputThrottle: 1.0,
		inputSteer:    0.0, // amount of steering input (-1.0..1.0)
	}

	vehicle.init()

	fps := 10.0
	secondPerFrame := 1.0 / fps

	for frame := 0; frame < 1000; frame++ {
		log.Println("ELAPSED TIME", float64(frame)*secondPerFrame, "s")
		compute(vehicle, secondPerFrame)
		time.Sleep(time.Millisecond * 100)
		vehicle.inputSteer += 0.01
	}
}

func sign(f float64) float64 {
	if f < 0.0 {
		return -1
	}
	return 1
}

func clamp(n float64, min float64, max float64) float64 {
	return math.Min(math.Max(n, min), max)
}

func compute(car *Car, dt float64) {

	steerAngle := car.inputSteer * car.maxSteer // actual front wheel steer angle (-maxSteer..maxSteer)

	// Pre-calc heading vector
	sn := math.Sin(car.heading)
	cs := math.Cos(car.heading)

	// Get velocity in local car coordinates
	car.velocity_c = car.velocity_c.
		SetX(cs*car.velocity.GetX() + sn*car.velocity.GetY()).
		SetY(cs*car.velocity.GetY() - sn*car.velocity.GetX())

	// Weight on axles based on centre of gravity and weight shift due to forward/reverse acceleration
	axleWeightFront := car.mass * (car.axleWeightRatioFront*car.gravity - car.weightTransfer*car.accel_c.GetX()*car.cgHeight/car.wheelBase)
	axleWeightRear := car.mass * (car.axleWeightRatioRear*car.gravity + car.weightTransfer*car.accel_c.GetX()*car.cgHeight/car.wheelBase)

	// Resulting velocity of the wheels as result of the yaw rate of the car body.
	// v = yawrate * r where r is distance from axle to CG and yawRate (angular velocity) in rad/s.
	yawSpeedFront := car.cgToFrontAxle * car.yawRate
	yawSpeedRear := -car.cgToRearAxle * car.yawRate

	// Calculate slip angles for front and rear wheels (a.k.a. alpha)
	slipAngleFront := math.Atan2(car.velocity_c.GetY()+yawSpeedFront, math.Abs(car.velocity_c.GetX())) - sign(car.velocity_c.GetX())*steerAngle
	slipAngleRear := math.Atan2(car.velocity_c.GetY()+yawSpeedRear, math.Abs(car.velocity_c.GetX()))

	tireGripFront := car.tireGrip
	tireGripRear := car.tireGrip * (1.0 - car.inputEbrake*(1.0-car.lockGrip)) // reduce rear grip when ebrake is on

	frictionForceFront_cy := clamp(-car.cornerStiffnessFront*slipAngleFront, -tireGripFront, tireGripFront) * axleWeightFront
	frictionForceRear_cy := clamp(-car.cornerStiffnessRear*slipAngleRear, -tireGripRear, tireGripRear) * axleWeightRear

	//  Get amount of brake/throttle from our inputs
	brake := math.Min(car.inputBrake*car.brakeForce+car.inputEbrake*car.eBrakeForce, car.brakeForce)
	throttleForce := car.inputThrottle * car.engineForce

	//  Resulting force in local car coordinates.
	//  This is implemented as a RWD car only.
	tractionForce_cx := throttleForce - brake*sign(car.velocity_c.GetX())
	tractionForce_cy := 0.0

	dragForce_cx := -car.rollResist*car.velocity_c.GetX() - car.airResist*car.velocity_c.GetX()*math.Abs(car.velocity_c.GetX())
	dragForce_cy := -car.rollResist*car.velocity_c.GetY() - car.airResist*car.velocity_c.GetY()*math.Abs(car.velocity_c.GetY())

	// total force in car coordinates
	totalForce_cx := dragForce_cx + tractionForce_cx
	totalForce_cy := dragForce_cy + tractionForce_cy + math.Cos(steerAngle)*frictionForceFront_cy + frictionForceRear_cy

	// acceleration along car axes
	car.accel_c = car.accel_c.
		SetX(totalForce_cx / car.mass). // forward/reverse accel
		SetY(totalForce_cy / car.mass)  // sideways accel

	// acceleration in world coordinates
	car.accel = car.accel.
		SetX(cs*car.accel_c.GetX() - sn*car.accel_c.GetY()).
		SetY(sn*car.accel_c.GetX() + cs*car.accel_c.GetY())

	// update velocity
	car.velocity = car.velocity.Add(vector.MakeVector2(
		car.accel.GetX()*dt,
		car.accel.GetY()*dt,
	))

	car.absVel = car.velocity.Mag()

	// calculate rotational forces
	angularTorque := (frictionForceFront_cy+tractionForce_cy)*car.cgToFrontAxle - frictionForceRear_cy*car.cgToRearAxle

	//  Sim gets unstable at very slow speeds, so just stop the car
	if math.Abs(car.absVel) < 0.5 && throttleForce == 0.0 {
		car.velocity = vector.MakeNullVector2()
		car.absVel = 0
		angularTorque = 0
		car.yawRate = 0
	}

	angularAccel := angularTorque / car.inertia

	car.yawRate += angularAccel * dt
	car.heading += car.yawRate * dt

	//  finally we can update position
	car.position = car.position.Add(vector.MakeVector2(
		car.velocity.GetX()*dt,
		car.velocity.GetY()*dt,
	))

	flags := uint(0)
	w := tabwriter.NewWriter(os.Stderr, 0, 4, 1, ' ', flags)
	fmt.Fprintln(w, "speed", car.velocity_c.GetX()*3600/1000, "\tKm/h")
	fmt.Fprintln(w, "accleration", car.accel_c.GetX())
	fmt.Fprintln(w, "yawRate", car.yawRate)
	fmt.Fprintln(w, "weightFront", axleWeightFront)
	fmt.Fprintln(w, "weightRear", axleWeightRear)
	fmt.Fprintln(w, "slipAngleFront", slipAngleFront)
	fmt.Fprintln(w, "slipAngleRear", slipAngleRear)
	fmt.Fprintln(w, "frictionFront", frictionForceFront_cy)
	fmt.Fprintln(w, "frictionRear", frictionForceRear_cy)
	w.Flush()

	/*
		//  Display some data
		car.stats.clear();  // clear this every tick otherwise it'll fill up fast
		car.stats.add('speed', car.velocity_c.x * 3600 / 1000 );  // km/h
		car.stats.add('accleration', car.accel_c.x);
		car.stats.add('yawRate', car.yawRate);
		car.stats.add('weightFront', axleWeightFront);
		car.stats.add('weightRear', axleWeightRear);
		car.stats.add('slipAngleFront', slipAngleFront);
		car.stats.add('slipAngleRear', slipAngleRear);
		car.stats.add('frictionFront', frictionForceFront_cy);
		car.stats.add('frictionRear', frictionForceRear_cy);
	*/
}
