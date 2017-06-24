package main

import (
	"log"

	"github.com/bytearena/bytearena/common/utils/vector"
)

func main() {
	log.Println("yooo")
}

// # Universal constants
const g = 9.80665 // g: Gravity, expressed in m/s2
const theta = 0.0 // θ: Angle of the slope in the direction of the motion (ground; 0 = no slope)

type UnitKilogram float64     // 1.0 = 1.0kg
type UnitMetre float64        // 1.0 = 1m
type UnitAcceleration float64 // 1.0 = 1m/s2
type UnitRPM float64          // 1.0 = 1.0 rotation per minute
type UnitTorque float64       // 1.0 = 1.0 N•m
type Force vector.Vector2

type Gear struct {
	Name  string
	Kind  int // -1: Reverse, 0: Neutral, 1: Forward
	Ratio float64
}

// # Vehicle constant Specs
type VehicleSpecs struct {
	Cdrag           float64                      // Cdrag: Drag constant of car, proportional to the frontal area of the car; for a Porsche, 0.31; for a car 0.4; for a truck 0.8
	Crr             float64                      // Crr: Rolling resistance constant; ranges from 0.01 to 0.02
	M               UnitKilogram                 // M: Mass of the vehicle, in kg
	Rwheel          UnitMetre                    // Rwheel: Wheel radius, in m.
	Cbraking        UnitAcceleration             // Cbraking: Braking force constant, expressed in m/s2
	MinRPM          UnitRPM                      // MinRPM: RPM when throttle = 0
	MaxRPM          UnitRPM                      // MaxRPM: RPM when throttle = 1
	TorqueCurve     func(rpm UnitRPM) UnitTorque // TorqueCurve: function of torque for RPM; expressed in N•m
	Gears           map[string]Gear              // gk (gR1, gF1, gF2, ..., gFn): Gear ratios for reverse and forward gears (gearbox ratio)
	FinalDriveRatio float64                      // G: Final Drive Ratio (differential ratio)
}

type VehicleStateControls struct {
	Braking  bool
	Throttle float64
	Gear     string
}

type VehicleStatePhysics struct {
}

type VehicleState struct {
	Controls VehicleStateControls
	Physics  VehicleStatePhysics
}
