package main

import (
	"fmt"
	"log"
	"math"
	"os"
	"text/tabwriter"
	"time"

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
			DragRatio:              0.30,
			FrontalArea:            1.50,
			RollingResistanceRatio: 0.015,
			Mass:                        1200.0,
			WheelRadius:                 0.34,
			Cbraking:                    10,
			MinRPM:                      1000,
			MaxRPM:                      7200,
			Gears:                       make(map[string]types.Gear, 0),
			FinalDriveRatio:             3.42,
			TransmissionEfficiencyRatio: 1.0, // 15% of energy loss in transmission
			EngineBrakingRatio:          0.74,

			WheelBase:                         2.416,
			CGFrontAxleWheelBaseDistanceRatio: 0.6,

			TorqueFunc: func(rpm unit.RPM) (torque unit.Torque, curveSlope float64 /* b */, base float64 /* d */) {

				b := 0.0
				d := 0.0

				if rpm <= 1000.0 {
					b = 0.0
					d = 280.0
				} else if rpm < 4600.0 {
					b = 0.025
					d = 280.0
				} else {
					b = -0.032
					d = 457.2
				}

				return b*rpm + d, b, d
			},
		},
	}

	vehicle.Specs.Gears["reverse"] = types.Gear{Name: "Reverse", Kind: -1, Ratio: 2.90}
	vehicle.Specs.Gears["neutral"] = types.Gear{Name: "Neutral", Kind: 0, Ratio: math.Inf(+1)}
	vehicle.Specs.Gears["forward1"] = types.Gear{Name: "Forward 1", Kind: 1, Ratio: 2.66}
	vehicle.Specs.Gears["forward2"] = types.Gear{Name: "Forward 2", Kind: 1, Ratio: 1.78}
	vehicle.Specs.Gears["forward3"] = types.Gear{Name: "Forward 3", Kind: 1, Ratio: 1.30}
	vehicle.Specs.Gears["forward4"] = types.Gear{Name: "Forward 4", Kind: 1, Ratio: 1.0}
	vehicle.Specs.Gears["forward5"] = types.Gear{Name: "Forward 5", Kind: 1, Ratio: 0.74}
	vehicle.Specs.Gears["forward6"] = types.Gear{Name: "Forward 6", Kind: 1, Ratio: 0.5}

	fps := 10.0
	secondPerFrame := 1.0 / fps

	for frame := 0; frame < 1000; frame++ {
		log.Println("ELAPSED TIME", float64(frame)*secondPerFrame, "s")
		compute(vehicle, secondPerFrame)
		if vehicle.State.Physics.RPM > 3500 {
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

func compute(vehicle *types.Vehicle, dt float64) {

	// # Universal constants
	const g = 9.80665 // g: Gravity, expressed in m/s2
	const airdensity = 1.2
	const theta = 0 // PI/2 = horizontal ground: Angle of the slope in the direction of the motion (ground; 0 = no slope)

	currentGearRatio := vehicle.Specs.Gears[vehicle.State.Controls.Gear].Ratio
	transmissionRatio := currentGearRatio * vehicle.Specs.FinalDriveRatio

	n := vehicle.Specs.TransmissionEfficiencyRatio
	Rw := vehicle.Specs.WheelRadius
	u := vehicle.State.Physics.Heading
	Ww := vehicle.State.Physics.WheelAngularVelocity
	v := vehicle.State.Physics.Velocity
	Crr := vehicle.Specs.RollingResistanceRatio
	A := vehicle.Specs.FrontalArea
	p := vehicle.State.Physics.Position
	M := vehicle.Specs.Mass
	speed := v.Mag()

	rpm := vehicle.State.Physics.RPM
	if rpm < 1000 {
		rpm = 1000
	}

	MaxEngineTorque, _, _ := vehicle.Specs.TorqueFunc(rpm)
	EngineTorque := MaxEngineTorque * vehicle.State.Controls.Throttle
	Tdrive := EngineTorque * transmissionRatio * n
	Fdrive := u.MultScalar(Tdrive / Rw)

	Ftraction := Fdrive

	Fdrag := v.MultScalar(-1 * 0.5 * vehicle.Specs.DragRatio * A * airdensity * math.Pow(speed, 2))
	Frr := v.MultScalar(-1 * Crr)
	Flong := Ftraction.Add(Fdrag).Add(Frr)
	a := Flong.DivScalar(M)

	vprime := v.Add(a.MultScalar(dt))
	pprime := p.Add(vprime.MultScalar(dt))
	Wwprime := vprime.Mag() / Rw

	vehicle.State.Physics.Velocity = vprime
	vehicle.State.Physics.Position = pprime
	vehicle.State.Physics.WheelAngularVelocity = Wwprime + math.Pi*2
	vehicle.State.Physics.Heading = vehicle.State.Physics.Heading
	vehicle.State.Physics.RPM = Wwprime * transmissionRatio * 60 / (math.Pi * 2)

	flags := uint(0)
	w := tabwriter.NewWriter(os.Stderr, 0, 4, 1, ' ', flags)
	fmt.Fprintln(w, "gear\t", vehicle.State.Controls.Gear, "\t")
	fmt.Fprintln(w, "RPM\t", number.ToFixed(rpm, 2), "\t")
	fmt.Fprintln(w, "EngineTorque\t", number.ToFixed(EngineTorque, 2), "\tN•m")
	fmt.Fprintln(w, "MaxEngineTorque\t", number.ToFixed(MaxEngineTorque, 2), "\tN•m")
	fmt.Fprintln(w, "DriveTorque\t", number.ToFixed(Tdrive, 2), "\tN•m")
	fmt.Fprintln(w, "DriveForce\t", number.ToFixed(Fdrive.Mag(), 2), "\tN")
	fmt.Fprintln(w, "DragForce\t", number.ToFixed(Fdrag.Mag(), 2), "\tN")
	fmt.Fprintln(w, "FrictionForce\t", number.ToFixed(Frr.Mag(), 2), "\tN")
	fmt.Fprintln(w, "Ww\t", number.ToFixed(Ww, 2), "\trad/s")
	fmt.Fprintln(w, "Ww\t", number.ToFixed(Ww/(math.Pi*2), 2), "\tRPS")
	fmt.Fprintln(w, "Ww\t", number.ToFixed((Ww/(math.Pi*2))*60, 2), "\tRPM")
	fmt.Fprintln(w, "acceleration\t", number.ToFixed(a.Mag(), 2), "\tm/s2")
	fmt.Fprintln(w, "velocity\t", number.ToFixed(vehicle.State.Physics.Velocity.Mag(), 2), "\tm/s")
	fmt.Fprintln(w, "velocity\t", number.ToFixed(vehicle.State.Physics.Velocity.Mag()*3.6, 2), "\tKm/h")
	// fmt.Fprintln(w, "W\t", number.ToFixed(W, 5), "\t")
	// fmt.Fprintln(w, "Wf\t", number.ToFixed(Wf, 5), "\t")
	// fmt.Fprintln(w, "Wr\t", number.ToFixed(Wr, 5), "\t")

	w.Flush()

	log.Println("===========================================")

}
