package main

import (
	"log"
	"math"

	"github.com/bytearena/bytearena/common/utils/number"
	"github.com/bytearena/bytearena/common/utils/vector"
	"github.com/netgusto/skeleton-car/common/unit"
)

func main() {

	vehicle := &Vehicle{
		State: &VehicleState{
			Controls: &VehicleStateControls{
				Brake:    0,
				Throttle: 1,
				Steering: 0,
				Gear:     "forward6",
			},
			Physics: &VehicleStatePhysics{
				Velocity: vector.MakeNullVector3(),
				Position: vector.MakeNullVector3(),
				Heading:  vector.MakeVector3(0, 1, 0),
			},
		},
		Specs: &VehicleSpecs{
			MaxSteering:            number.DegreeToRadian(45),
			DragRatio:              0.31,
			FrontalArea:            1.94,
			RollingResistanceRatio: 0.015,
			Mass:               1393.0,
			WheelRadius:        0.3186,
			Cbraking:           10,
			MinRPM:             1000,
			MaxRPM:             7200,
			Gears:              make(map[string]Gear, 0),
			FinalDriveRatio:    3.44,
			EngineBrakingRatio: 0.74,
			TorqueCurve: func(rpm unit.RPM) unit.Torque {
				if rpm <= 1000 {
					return 220
				}

				if rpm < 4600 {
					return 0.025*rpm + 195
				}

				return -0.032*rpm + 457.2
			},
		},
	}

	vehicle.Specs.Gears["reverse"] = Gear{Name: "Reverse", Kind: -1, Ratio: 3.82}
	vehicle.Specs.Gears["neutral"] = Gear{Name: "Neutral", Kind: 0, Ratio: math.Inf(+1)}
	vehicle.Specs.Gears["forward1"] = Gear{Name: "Forward 1", Kind: 1, Ratio: 3.82}
	vehicle.Specs.Gears["forward2"] = Gear{Name: "Forward 2", Kind: 1, Ratio: 2.20}
	vehicle.Specs.Gears["forward3"] = Gear{Name: "Forward 3", Kind: 1, Ratio: 1.52}
	vehicle.Specs.Gears["forward4"] = Gear{Name: "Forward 4", Kind: 1, Ratio: 1.22}
	vehicle.Specs.Gears["forward5"] = Gear{Name: "Forward 5", Kind: 1, Ratio: 1.02}
	vehicle.Specs.Gears["forward6"] = Gear{Name: "Forward 6", Kind: 1, Ratio: 0.84}

	compute(vehicle)
}

// # Universal constants
const g = 9.80665 // g: Gravity, expressed in m/s2
const airdensity = 1.2
const theta = 0 // PI/2 = horizontal ground: Angle of the slope in the direction of the motion (ground; 0 = no slope)

type Gear struct {
	Name  string
	Kind  int // -1: Reverse, 0: Neutral, 1: Forward
	Ratio unit.Ratio
}

// # Vehicle constant Specs
type VehicleSpecs struct {
	MaxSteering            unit.Angle                     // Maximum angle of a wheel relative to the longitudinal axis of the vehicle (assumed symetrical in both directions), 0 being the wheel aligned with the longitudinal axis of the vehicle; expressed in radians
	DragRatio              unit.Ratio                     // Cdrag: Drag constant of car, proportional to the frontal area of the car; for a Porsche, 0.31; for a car 0.4; for a truck 0.8
	FrontalArea            unit.SquareMetre               // Frontal Area expressed in m2; simplification as it's not always the front of the car that's facing the movement
	RollingResistanceRatio unit.Ratio                     // Crr: Rolling resistance constant; ranges from 0.01 to 0.02
	Mass                   unit.Kilogram                  // M: Mass of the vehicle, in kg
	WheelRadius            unit.Metre                     // Rwheel: Wheel radius, in m.
	Cbraking               unit.Acceleration              // Cbraking: Braking force constant, expressed in m/s2
	MinRPM                 unit.RPM                       // MinRPM: RPM when throttle = 0
	MaxRPM                 unit.RPM                       // MaxRPM: RPM when throttle = 1
	TorqueCurve            func(rpm unit.RPM) unit.Torque // TorqueCurve: function of torque for RPM; expressed in N•m
	Gears                  map[string]Gear                // gk (gR1, gF1, gF2, ..., gFn): Gear ratios for reverse and forward gears (gearbox ratio)
	FinalDriveRatio        unit.Ratio                     // G: Final Drive Ratio (differential ratio)
	EngineBrakingRatio     unit.Ratio                     // Braking force coefficient exerted by the frictions in the engine
	MaxBrakingAcceleration unit.Acceleration              // Maximum braking acceleration (deceleration in fact) exerted by the vehicle brakes, expressed in m/s2
}

type VehicleStateControls struct {
	Brake    unit.IntervalUnit         // 0: not braking; 1: full braking
	Throttle unit.IntervalUnit         // 0: no throttle; 1: full throttle
	Steering unit.SymetricIntervalUnit // Rotation of the steering wheels on their vertical axis; -1: full left; 0: straight ahead; 1: full right
	Gear     string
}

type VehicleStatePhysics struct {
	Velocity unit.Force
	Position unit.Point
	Heading  unit.Vector // Unit vector indicating where the nose points (not necessarily the direction of the movement); Orientation of the longitudinal axis of the vehicle, from back to front
}

type VehicleState struct {
	Controls *VehicleStateControls
	Physics  *VehicleStatePhysics
}

type Vehicle struct {
	State *VehicleState
	Specs *VehicleSpecs
}

func compute(vehicle *Vehicle) {
	currentGearRatio := vehicle.Specs.Gears[vehicle.State.Controls.Gear].Ratio
	transmissionRatio := currentGearRatio * vehicle.Specs.FinalDriveRatio

	brakingAcceleration := unit.Acceleration(number.Map(vehicle.State.Controls.Brake, 0, 1, 0, vehicle.Specs.MaxBrakingAcceleration))
	engineRPM := unit.RPM(number.Map(vehicle.State.Controls.Throttle, 0, 1, vehicle.Specs.MinRPM, vehicle.Specs.MaxRPM))

	engineTorque := vehicle.Specs.TorqueCurve(engineRPM)
	engineBrakingTorque := vehicle.Specs.EngineBrakingRatio * (engineRPM / 60) * (1 - vehicle.State.Controls.Throttle)
	netEngineTorque := engineTorque

	wheelsTorque := netEngineTorque * transmissionRatio

	tiresForce := wheelsTorque / vehicle.Specs.WheelRadius
	gravityDownwardsForce := vehicle.Specs.Mass * g * math.Cos(theta)
	gravityBackwardsForce := vehicle.Specs.Mass * g * math.Sin(theta)
	rollingResistanceForce := gravityDownwardsForce * vehicle.Specs.RollingResistanceRatio
	aerodynamicDragForce := 0.5 * vehicle.Specs.DragRatio * vehicle.State.Physics.Velocity.MagSq() * vehicle.Specs.FrontalArea

	totalForce := tiresForce - (rollingResistanceForce + gravityBackwardsForce + aerodynamicDragForce)
	acceleration := totalForce / vehicle.Specs.Mass

	wheelAngularVelocity := unit.AngularVelocity(2.0 * math.Pi * engineRPM / (60.0 * transmissionRatio))
	wheelLinearVelocity := wheelAngularVelocity * vehicle.Specs.WheelRadius

	// TODO: engine braking, braking, cornering, weight distribution

	log.Println("\n",
		"engineRPM", number.ToFixed(engineRPM, 5), "\n",
		"engineTorque", number.ToFixed(engineTorque, 5), "\n",
		"engineBrakingTorque", number.ToFixed(engineBrakingTorque, 5), "\n",
		"netEngineTorque", number.ToFixed(netEngineTorque, 5), "\n",
		"transmissionRatio", number.ToFixed(transmissionRatio, 5), "\n",
		"wheelsTorque", number.ToFixed(wheelsTorque, 5), "\n",
		"tiresForce", number.ToFixed(tiresForce, 5), "\n",
		"gravityDownwardsForce", number.ToFixed(gravityDownwardsForce, 5), "\n",
		"gravityBackwardsForce", number.ToFixed(gravityBackwardsForce, 5), "\n",
		"rollingResistanceForce", number.ToFixed(rollingResistanceForce, 5), "\n",
		"aerodynamicDragForce", number.ToFixed(aerodynamicDragForce, 5), "\n",
		"totalForce", number.ToFixed(totalForce, 5), "\n",
		"acceleration", number.ToFixed(acceleration, 5), "\n",
		"wheelAngularVelocity (rad/s)", number.ToFixed(wheelAngularVelocity, 5), "\n",
		"wheelLinearVelocity (m/s)", number.ToFixed(wheelLinearVelocity, 5), "\n",
		"wheelLinearVelocity (km/h)", number.ToFixed(wheelLinearVelocity*3.6, 5), "\n",
		"brakingAcceleration", number.ToFixed(brakingAcceleration, 5), "\n",
	)
}

/*
// Vector. Resistance of the air. Rises to the square of the velocity, in the inverse direction of the velocity
func computeAerodynamicDragForce(vehicle *Vehicle) unit.Force {
	// Fdrag = -Cdrag * V * |V|
	return vehicle.State.Physics.Velocity.MultScalar(-1 * vehicle.Specs.DragRatio * vehicle.State.Physics.Velocity.Mag())
}

// Vector. Resistance of the ground.
func computeRollingResistanceForce(vehicle *Vehicle) unit.Force {
	return vehicle.State.Physics.Velocity.MultScalar(-1 * vehicle.Specs.RollingResistanceRatio)
}

// Vector. Always pointing downwards; theta (ground angle) is always 0
func computeGravityForce(vehicle *Vehicle) unit.Force {
	// theta for horizontal ground is PI/2; sin(PI/2) = 1; an optimization is then to always use *1 instead of *math.Sin(theta)
	// return vector.MakeVector3(0, 0, -1).MultScalar(vehicle.Specs.Mass * g * math.Sin(theta))
	return vector.MakeVector3(0, 0, -1).MultScalar(vehicle.Specs.Mass * g)
}

// Value in rotations/minute. RPM of the engine.
func computeEngineRPM(vehicle *Vehicle) unit.RPM {
	return number.Map(vehicle.State.Controls.Throttle, 0, 1, vehicle.Specs.MinRPM, vehicle.Specs.MaxRPM)
}

// Value in N•M. Torque of the engine, applied to the gears
func computeEngineTorque(vehicle *Vehicle) unit.Torque {
	engineRpm := computeEngineRPM(vehicle)
	return vehicle.Specs.TorqueCurve(engineRpm)
}

// Value in rad/s. Angular velocity of the engine.
func computeEngineAngularVelocity(vehicle *Vehicle) unit.AngularVelocity {
	return math.Pi * 2 * computeEngineRPM(vehicle) / 60 // 60: expressed in rad/s
}

// Value in Watts. Power developed by the engine.
func computeEnginePower(vehicle *Vehicle) unit.Watt {
	return computeEngineAngularVelocity(vehicle) * computeEngineTorque(vehicle)
}

// Value in N•m. Torque applied to the wheels
func computeWheelTorque(vehicle *Vehicle) unit.Torque {
	return computeEngineTorque(vehicle) * computeDriveTrainRatio(vehicle)
}

// Numeric value. Total ratio of the vehicle drivetrain (current gear ratio and final drive ratio)
func computeDriveTrainRatio(vehicle *Vehicle) unit.Ratio {
	currentGearRatio := 1.0
	return currentGearRatio * vehicle.Specs.FinalDriveRatio
}

// Value in rad/s. Angular velocity of the wheels.
func computeWheelAngularVelocity(vehicle *Vehicle) unit.AngularVelocity {
	drivetrainRatio := computeDriveTrainRatio(vehicle)
	if drivetrainRatio == 0.0 { // Neutral Gear ?
		return 0.0
	}
	return computeEngineAngularVelocity(vehicle) / drivetrainRatio
}

// Value in m/s. Linear velocity of the car (assuming perfect rolling, no slipping of the wheels)
func computeWheelLinearVelocity(vehicle *Vehicle) unit.Force {
	// quantity of movement (no slipping in the direction of movement => 1:1 ratio of movement)
	meterPerSecond := vehicle.Specs.WheelRadius * computeWheelAngularVelocity(vehicle)

	///////////////////////////////////////////////////////////////////////////
	// direction of movement (no slipping on the side of the whees => 1:1 ratio of wheel direction)
	///////////////////////////////////////////////////////////////////////////

	// Transform Controls.Steering in an angle relative to longitudinal axis
	// 1. Map Controls.Steering on steering range [-MaxSteering, MaxSteering]
	steeringAngle := number.Map(vehicle.State.Controls.Steering, -1, 1, -vehicle.Specs.MaxSteering, vehicle.Specs.MaxSteering)

	// 2. Make vector relative to the vehicle longitudinal axis with that angle, and set quantity of movement
	return vector.
		MakeVector3(0, 1, 0).           // unit vector pointing North (Y)
		SetAngleOnZAxis(steeringAngle). // oriented
		SetMag(meterPerSecond)          // and finally set to the length corresponding to the quantity of movement
}

func computeTractionForce(vehicle *Vehicle) unit.Force {
	// TODO: what is this ? Is it really a vector ?
	// Vector. Friction between the tires and the ground  resulting in a force applied in the direction opposite to the rotation of the tires
	// Ftraction = Twheel / Rwheel

	// quantity of force
	meterPerSecond := computeWheelTorque(vehicle) / vehicle.Specs.WheelRadius

	steeringAngle := number.Map(vehicle.State.Controls.Steering, -1, 1, -vehicle.Specs.MaxSteering, vehicle.Specs.MaxSteering)

	return vector.
		MakeVector3(0, 1, 0).           // unit vector pointing North (Y)
		SetAngleOnZAxis(steeringAngle). // oriented
		SetMag(meterPerSecond)          // and finally set to the length corresponding to the quantity of movement
}

func computeBrakingForce(vehicle *Vehicle) unit.Force {
	//return vehicle.State.Physics.vehicle.State.Controls.Brake
	return vector.MakeNullVector3()
}
*/
