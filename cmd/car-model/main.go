package main

import (
	"fmt"
	"math"
	"os"
	"text/tabwriter"
	"time"

	"log"

	"github.com/bytearena/bytearena/common/utils/number"
	"github.com/bytearena/bytearena/common/utils/vector"
	"github.com/netgusto/skeleton-car/common/types"
	"github.com/netgusto/skeleton-car/common/unit"
)

func main() {

	vehicle := &types.Vehicle{
		State: &types.VehicleState{
			Controls: &types.VehicleStateControls{
				Brake:    0,
				Throttle: 1,
				Steering: 0,
				Gear:     "forward1",
			},
			Physics: &types.VehicleStatePhysics{
				Velocity:             vector.MakeNullVector2(),
				Position:             vector.MakeNullVector2(),
				Heading:              vector.MakeVector2(0, 1),
				RPM:                  0,
				WheelAngularVelocity: 0,
			},
		},
		Specs: &types.VehicleSpecs{
			MaxSteering:            number.DegreeToRadian(45),
			DragRatio:              0.31,
			FrontalArea:            1.94,
			RollingResistanceRatio: 0.015,
			Mass:               1393.0,
			WheelRadius:        0.3186,
			Cbraking:           10,
			MinRPM:             1000,
			MaxRPM:             7200,
			Gears:              make(map[string]types.Gear, 0),
			FinalDriveRatio:    3.44,
			EngineBrakingRatio: 0.74,
			TorqueFunc: func(rpm unit.RPM) (torque unit.Torque, curveSlope float64 /* b */, base float64 /* d */) {

				b := 0.0
				d := 0.0

				if rpm <= 1000.0 {
					b = 0.0
					d = 220.0
				} else if rpm < 4600.0 {
					b = 0.025
					d = 195.0
				} else {
					b = -0.032
					d = 457.2
				}

				return b*rpm + d, b, d
			},
		},
	}

	vehicle.Specs.Gears["reverse"] = types.Gear{Name: "Reverse", Kind: -1, Ratio: 3.82}
	vehicle.Specs.Gears["neutral"] = types.Gear{Name: "Neutral", Kind: 0, Ratio: math.Inf(+1)}
	vehicle.Specs.Gears["forward1"] = types.Gear{Name: "Forward 1", Kind: 1, Ratio: 3.82}
	vehicle.Specs.Gears["forward2"] = types.Gear{Name: "Forward 2", Kind: 1, Ratio: 2.20}
	vehicle.Specs.Gears["forward3"] = types.Gear{Name: "Forward 3", Kind: 1, Ratio: 1.52}
	vehicle.Specs.Gears["forward4"] = types.Gear{Name: "Forward 4", Kind: 1, Ratio: 1.22}
	vehicle.Specs.Gears["forward5"] = types.Gear{Name: "Forward 5", Kind: 1, Ratio: 1.02}
	vehicle.Specs.Gears["forward6"] = types.Gear{Name: "Forward 6", Kind: 1, Ratio: 0.84}

	fps := 10.0
	secondPerFrame := 1.0 / fps

	for frame := 0; frame < 10000; frame++ {
		log.Println("ELAPSED TIME", float64(frame)*secondPerFrame, "s")
		compute(vehicle, secondPerFrame)
		if vehicle.State.Physics.RPM > vehicle.Specs.MaxRPM-500 {
			nextGear := vehicle.State.Controls.Gear
			switch nextGear {
			case "forward1":
				nextGear = "forward2"
				break
			case "forward2":
				nextGear = "forward3"
				break
			case "forward3":
				nextGear = "forward4"
				break
			case "forward4":
				nextGear = "forward5"
				break
			case "forward5":
				nextGear = "forward6"
				break
			}

			vehicle.State.Controls.Gear = nextGear
		}
		time.Sleep(time.Millisecond * 100)
	}
}

// # Universal constants
const g = 9.80665 // g: Gravity, expressed in m/s2
const airdensity = 1.2
const theta = 0 // PI/2 = horizontal ground: Angle of the slope in the direction of the motion (ground; 0 = no slope)

func compute(vehicle *types.Vehicle, dt float64) {

	currentGearRatio := vehicle.Specs.Gears[vehicle.State.Controls.Gear].Ratio
	transmissionRatio := currentGearRatio * vehicle.Specs.FinalDriveRatio

	// Force = m * a
	//  => a = Force / m
	//  => m = Force / a

	// F = symbol for weight, measured in Newtons, N.
	// m = symbol for mass, measured in kilograms, or kg.
	// a = symbol for acceleration, expressed as m/s2, or meters per second squared

	brakingAcceleration := unit.Acceleration(number.Map(vehicle.State.Controls.Brake, 0, 1, 0, vehicle.Specs.MaxBrakingAcceleration)) // in m/s2
	engineRPM := vehicle.State.Physics.RPM

	engineTorque, _, _ := vehicle.Specs.TorqueFunc(engineRPM)                                                          // in N•m
	engineBrakingTorque := vehicle.Specs.EngineBrakingRatio * (engineRPM / 60) * (1 - vehicle.State.Controls.Throttle) // in N•m

	netEngineTorque := engineTorque // in N•m
	var wheelsTorque unit.Torque
	if math.IsInf(transmissionRatio, +1) { // happens when in neutral gear
		wheelsTorque = 0 // in N•m
	} else {
		wheelsTorque = netEngineTorque * transmissionRatio // in N•m
	}

	wheelsForce := wheelsTorque / vehicle.Specs.WheelRadius                                // in Newtons
	gravityDownwardsForce := vehicle.Specs.Mass * g * math.Cos(theta)                      // in Newtons
	gravityBackwardsForce := vehicle.Specs.Mass * g * math.Sin(theta)                      // in Newtons
	rollingResistanceForce := gravityDownwardsForce * vehicle.Specs.RollingResistanceRatio // in Newtons

	aerodynamicDragForce := 0.5 * vehicle.Specs.DragRatio * vehicle.State.Physics.Velocity.MagSq() * vehicle.Specs.FrontalArea // in Newtons

	var totalForce float64                    // in Newtons
	if rollingResistanceForce > wheelsForce { // rolling resistance is bigger than tires force; the vehicle is not moving
		// TODO: this will be wrong when ground slope is != 0, as the gravityBackwardsForce might pull the car backwards, resulting in a non null force
		// For now, ground slope is null; plus, the car use it's parking brake when stopped !
		totalForce = 0 // in Newtons
	} else {
		totalForce = wheelsForce - (rollingResistanceForce + gravityBackwardsForce + aerodynamicDragForce) // in Newtons
	}

	wheelTheoricExertedAcceleration := totalForce / vehicle.Specs.Mass // in m/s2 (a = Force / m); not drag bound

	wheelAngularVelocity := unit.AngularVelocity(2.0 * math.Pi * engineRPM / (60.0 * transmissionRatio)) // in rad/s
	wheelVelocity := wheelAngularVelocity * vehicle.Specs.WheelRadius                                    // in m/s, without drag limitiation

	// Determining vehicle linear velocity depending on wheelLinearVelocity and aerodynamic drag

	_, rpmCurveSlope, rpmCurveBase := vehicle.Specs.TorqueFunc(engineRPM) // in N•m

	// Implementing equation 8.21

	c1 := -0.5 * ((vehicle.Specs.DragRatio * airdensity * vehicle.Specs.FrontalArea) / vehicle.Specs.Mass)
	c2 := (60 * (currentGearRatio * currentGearRatio) * (vehicle.Specs.FinalDriveRatio * vehicle.Specs.FinalDriveRatio) * rpmCurveSlope) / (2.0 * math.Pi * vehicle.Specs.Mass * (vehicle.Specs.WheelRadius * vehicle.Specs.WheelRadius))
	c3 := ((currentGearRatio * vehicle.Specs.FinalDriveRatio * rpmCurveBase) / (vehicle.Specs.Mass * vehicle.Specs.WheelRadius)) - (vehicle.Specs.RollingResistanceRatio * g * math.Cos(theta)) - (g * math.Sin(theta))

	var vehicleAcceleration float64 // in m/s2
	var vmax float64                // in m/s
	if math.IsInf(c2, -1) || math.IsInf(c3, +1) {
		vehicleAcceleration = 0.0
	} else {
		vehicleAcceleration = (c1 * (wheelVelocity * wheelVelocity)) + (c2 * wheelVelocity) + c3

		// Implementing equation 8.23
		// Solving a = c1 * v^2 + c2 * v + c3 = 0

		vmax1 := (-c2 + math.Sqrt(math.Pow(c2, 2)-4*c1*c3)) / (2 * c1)
		vmax2 := (-c2 - math.Sqrt(math.Pow(c2, 2)-4*c1*c3)) / (2 * c1)
		vmax = math.Max(vmax1, vmax2) // Two solutions, one negative, one positive
	}

	integratedVelocity := vehicle.State.Physics.Heading.Clone().SetMag(dt * vehicleAcceleration)
	vehicleVelocityPrime := vehicle.State.Physics.Velocity.Add(integratedVelocity)
	vehiclePositionPrime := vehicle.State.Physics.Position.Clone().Add(vehicleVelocityPrime.MultScalar(dt))

	wheelAngularVelocityPrime := vehicleVelocityPrime.Mag() / vehicle.Specs.WheelRadius
	RPMPrime := wheelAngularVelocityPrime * transmissionRatio * 60 / (math.Pi * 2)

	// TODO: engine braking, braking, cornering, weight distribution, wheel traction

	flags := uint(0)
	w := tabwriter.NewWriter(os.Stderr, 0, 4, 1, ' ', flags)

	if false {
		fmt.Fprintln(w, "throttle\t", number.ToFixed(vehicle.State.Controls.Throttle, 5), "\t")
		fmt.Fprintln(w, "steering\t", number.ToFixed(vehicle.State.Controls.Steering, 5), "\t")
		fmt.Fprintln(w, "brake\t", number.ToFixed(vehicle.State.Controls.Brake, 5), "\t")
		fmt.Fprintln(w, "transmissionRatio\t", number.ToFixed(transmissionRatio, 5), "\t")
		fmt.Fprintln(w, "")
		fmt.Fprintln(w, "engineTorque\t", number.ToFixed(engineTorque, 5), "\tN•m")
		fmt.Fprintln(w, "engineBrakingTorque\t", number.ToFixed(engineBrakingTorque, 5), "\tN•m")
		fmt.Fprintln(w, "netEngineTorque\t", number.ToFixed(netEngineTorque, 5), "\tN•m")
		fmt.Fprintln(w, "wheelsTorque\t", number.ToFixed(wheelsTorque, 5), "\tN•m")
		fmt.Fprintln(w, "")
		fmt.Fprintln(w, "wheelsForce\t", number.ToFixed(wheelsForce, 5), "\tN")
		fmt.Fprintln(w, "gravityDownwardsForce\t", number.ToFixed(gravityDownwardsForce, 5), "\tN")
		fmt.Fprintln(w, "gravityBackwardsForce\t", number.ToFixed(gravityBackwardsForce, 5), "\tN")
		fmt.Fprintln(w, "rollingResistanceForce\t", number.ToFixed(rollingResistanceForce, 5), "\tN")
		fmt.Fprintln(w, "aerodynamicDragForce\t", number.ToFixed(aerodynamicDragForce, 5), "\tN")
		fmt.Fprintln(w, "totalForce\t", number.ToFixed(totalForce, 5), "\tN")
		fmt.Fprintln(w, "")
		fmt.Fprintln(w, "wheelAngularVelocity\t", number.ToFixed(wheelAngularVelocity, 5), "\trad/s")
		fmt.Fprintln(w, "wheelVelocity\t", number.ToFixed(wheelVelocity, 5), "\tm/s")
		fmt.Fprintln(w, "wheelVelocity\t", number.ToFixed(wheelVelocity*3.6, 5), "\tKm/h")
		fmt.Fprintln(w, "brakingAcceleration\t", number.ToFixed(brakingAcceleration, 5), "\tm/s2")
		fmt.Fprintln(w, "wheelTheoricAcceleration\t", number.ToFixed(wheelTheoricExertedAcceleration, 5), "\tm/s2", "\t(not drag bound)")
	}

	fmt.Fprintln(w, "gear\t", vehicle.State.Controls.Gear, "\t")
	fmt.Fprintln(w, "WheelAngularVelocity\t", number.ToFixed(wheelAngularVelocity, 5), "\trad/s")
	fmt.Fprintln(w, "WheelAngularVelocity\t", number.ToFixed((wheelAngularVelocity/(math.Pi*2))*60, 5), "\tRPM")
	fmt.Fprintln(w, "engineRPM\t", number.ToFixed(engineRPM, 5), "\tRPM")
	fmt.Fprintln(w, "vehicleAcceleration\t", vehicleAcceleration, "\tm/s2", "\t(drag bound)")
	fmt.Fprintln(w, "vehicleVelocity\t", vehicleVelocityPrime.Mag(), "\tm/s", "\t(force)")
	fmt.Fprintln(w, "vehicleVelocity\t", vehicleVelocityPrime.Mag()*3.6, "\tKm/h", "\t(force)")
	fmt.Fprintln(w, "vehicleMaxVelocity\t", vmax, "\tm/s", "\t(drag bound; in current gear, at current RPM)")
	fmt.Fprintln(w, "vehicleMaxVelocity\t", vmax*3.6, "\tKm/h", "\t(drag bound; in current gear, at current RPM)")
	//fmt.Fprintln(w, "vehiclePosition\t", vehiclePosition, "\t", "\t")
	w.Flush()

	log.Println("===========================================")

	vehicle.State.Physics.Position = vehiclePositionPrime
	vehicle.State.Physics.Velocity = vehicleVelocityPrime
	vehicle.State.Physics.WheelAngularVelocity = wheelAngularVelocityPrime
	vehicle.State.Physics.RPM = RPMPrime
}
