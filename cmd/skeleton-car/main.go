package main

import (
	"log"
	"math"
)

type Motion struct {
	Angle    float64
	Distance float64
}

type Point struct {
	X float64
	Y float64
}

func main() {

	length := 20.0
	motions := []Motion{Motion{0, 10}, Motion{math.Pi / 6, 10}, Motion{0, 20}}

	position := Point{0.0, 0.0}
	orientation := 0.0
	for _, motion := range motions {
		position, orientation = bicycleMove(motion, position, orientation, length)
		log.Println(position, orientation)
	}
}

func bicycleMove(motion Motion, position Point, absoluteangle float64, lengthBetweenAxles float64) (Point, float64) {

	// See https://www.youtube.com/watch?v=5lWj1FMkq5I

	// Implements a bicycle model drivetrain (2 wheels, or 4 wheels, left and right approximated parallel)
	// Does not implement any physics (movement is not constrained to the laws of physics)

	// Given:
	// x & y       : Position point of center of the rear axle
	// L		   : Length between front wheel and rear wheel axles
	// d           : Distance of movement of the vehicle
	// α (alpha)   : Angle of front wheel **relative** to center line of the vehicle
	// θ (theta)   : Angle of center line of the vehicle, **absolute**

	// We find:
	// β (beta)	   : Turning angle on turning circle
	// R           : Radius of turning circle
	// Cx & Cy     : Center point of turning circle
	// x' & y'     : New position point of center of the rear axle
	// θ'          : New angle of center line of the vehicle, **absolute**

	x := position.X
	y := position.Y
	L := lengthBetweenAxles
	theta := absoluteangle

	var xprime float64
	var yprime float64
	var thetaprime float64

	d := motion.Distance
	alpha := motion.Angle

	beta := (d / L) * math.Tan(alpha)

	if beta < 0.001 {
		// linear movement approximation
		xprime = x + d*math.Cos(theta)
		yprime = y + d*math.Sin(theta)
		thetaprime = math.Mod(theta+beta, 2*math.Pi)
	} else {
		R := d / beta

		Cx := x - math.Sin(theta)*R
		Cy := y + math.Cos(theta)*R
		xprime = Cx + math.Sin(theta+beta)*R
		yprime = Cy - math.Cos(theta+beta)*R
		thetaprime = math.Mod(theta+beta, 2*math.Pi)
	}

	return Point{xprime, yprime}, thetaprime
}
