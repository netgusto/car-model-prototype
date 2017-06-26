package unit

import "github.com/bytearena/bytearena/common/utils/vector"

type Kilogram = float64 // 1.0 = 1.0kg
type RPM = float64      // 1.0 = 1.0 rotation per minute
type Torque = float64   // 1.0 = 1.0 N•m
type Metre = float64    // 1.0 = 1m
//type UnitLinearVelocity = float64  // 1.0 = 1m/s
type AngularVelocity = float64 // 1.0 = 1 rad/s
type Acceleration = float64    // 1.0 = 1m/s2
type Watt = float64            // 1.0 = 1 watt
type Angle = float64           // 1.0 = 1 rad
type SquareMetre = float64     // 1.0 = 1m2

type Vector = vector.Vector3
type Force = Vector
type Point = Vector

type IntervalUnit = float64         // float in [0,1]
type SymetricIntervalUnit = float64 // float in [-1,1]
type Ratio = float64
