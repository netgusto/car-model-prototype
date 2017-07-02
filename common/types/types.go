package types

import "github.com/bytearena/car-model-prototype/common/unit"

type Gear struct {
	Name  string
	Kind  int // -1: Reverse, 0: Neutral, 1: Forward
	Ratio unit.Ratio
}

// # Vehicle constant Specs
type VehicleSpecs struct {
	MaxSteering                 unit.Angle        // Maximum angle of a wheel relative to the longitudinal axis of the vehicle (assumed symetrical in both directions), 0 being the wheel aligned with the longitudinal axis of the vehicle; expressed in radians
	DragRatio                   unit.Ratio        // Cdrag: Drag constant of car, proportional to the frontal area of the car; for a Porsche, 0.31; for a car 0.4; for a truck 0.8
	FrontalArea                 unit.SquareMetre  // Frontal Area expressed in m2; simplification as it's not always the front of the car that's facing the movement
	RollingResistanceRatio      unit.Ratio        // Crr: Rolling resistance constant; ranges from 0.01 to 0.02
	Mass                        unit.Kilogram     // M: Mass of the vehicle, in kg
	WheelRadius                 unit.Metre        // Rwheel: Wheel radius, in m.
	Cbraking                    unit.Acceleration // Cbraking: Braking force constant, expressed in m/s2
	MinRPM                      unit.RPM          // MinRPM: RPM when throttle = 0
	MaxRPM                      unit.RPM          // MaxRPM: RPM when throttle = 1
	Gears                       map[string]Gear   // gk (gR1, gF1, gF2, ..., gFn): Gear ratios for reverse and forward gears (gearbox ratio)
	FinalDriveRatio             unit.Ratio        // G: Final Drive Ratio (differential ratio)
	EngineBrakingRatio          unit.Ratio        // Braking force coefficient exerted by the frictions in the engine
	TransmissionEfficiencyRatio unit.Ratio        // Ratio of efficiency of the transmission (1 = perfect transmission)
	MaxBrakingAcceleration      unit.Acceleration // Maximum braking acceleration (deceleration in fact) exerted by the vehicle brakes, expressed in m/s2
	MaxTireAccelerationOnG      unit.Acceleration // Maximum Tire acceleration (muK)

	WheelBase                         unit.Metre // Distance between front axle and rear axle
	CGFrontAxleWheelBaseDistanceRatio unit.Ratio // Ratio of Distance between the center of gravity and the front axle (relative to WheelBase)
	CGHeight                          unit.Metre // Height position of the CG, in metres

	TorqueFunc func(rpm unit.RPM) (torque unit.Torque, curveSlope float64 /* b */, base float64 /* d */) // TorqueFunc: function of torque for RPM; expressed in N•m
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
	Heading  unit.Vector // Unit vector indicating directional wheel orientation, relative to the car

	RPM                  unit.RPM
	WheelAngularVelocity unit.AngularVelocity
}

type VehicleState struct {
	Controls *VehicleStateControls
	Physics  *VehicleStatePhysics
}

type Vehicle struct {
	State *VehicleState
	Specs *VehicleSpecs
}
