package accumulator

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

type D struct {
	// Force is the radial force component of a force vector.
	Force float64

	// Torque may be calculated via
	//
	//   T = || r x F || => ||T|| = rF * sin(𝜃)
	//
	// Where F is the input force. Note that this is in essence just
	// factoring in the tangential force relative to the agent.
	Torque float64
}

type A struct {
	limit       D
	accumulator D

	heading polar.V
	radius  float64
}

func New(limit D, radius float64, heading polar.V) *A {
	return &A{
		limit:       limit,
		accumulator: D{},
		heading:     heading,
		radius:      radius,
	}
}

// Add truncates the given input vector by the maximum change limits.  If the
// given input vector will exceed the given max, the limit bool return value
// will be set to false.
func (a *A) Add(force vector.V) (vector.V, bool) {
	if vector.Within(force, *vector.New(0, 0)) {
		return *vector.New(0, 0), true
	}

	// theta is the relative angle directed from the heading to the force.
	theta := polar.Sub(
		polar.Polar(force),
		a.heading,
	).Theta()

	delta := D{
		Force:  vector.Magnitude(force) * math.Cos(theta),
		Torque: a.radius * vector.Magnitude(force) * math.Sin(theta),
	}

	remainder := D{
		Force:  math.Max(0, a.limit.Force-a.accumulator.Force),
		Torque: math.Max(0, a.limit.Torque-a.accumulator.Torque),
	}

	df := D{
		Force: math.Copysign(
			math.Min(math.Abs(delta.Force), remainder.Force),
			delta.Force,
		),
		Torque: math.Copysign(
			math.Min(math.Abs(delta.Torque), remainder.Torque),
			delta.Torque,
		),
	}

	a.accumulator = D{
		Force:  a.accumulator.Force + math.Abs(df.Force),
		Torque: a.accumulator.Torque + math.Abs(df.Torque),
	}

	// rtheta is the relative angle between the returned force vector and
	// the agent heading.
	rtheta := math.Atan2(df.Torque, a.radius*df.Force)

	// rforce is the resultant force vector relative to the agent heading.
	rforce := *polar.New(
		// Note that
		//
		//   F = F_r / cos(𝜃) = T / r / sin(𝜃)
		//
		// At the extrema of sin and cos, we can use the other as a
		// fallback, ensuring we always get a sensible net force value
		// for all angles.
		map[bool]float64{
			true:  df.Force / math.Cos(rtheta),
			false: df.Torque / a.radius / math.Sin(rtheta),
			// We are picking a relatively large cutoff branching value
			// (compared to absolute floating precision) because dividing by
			// very small numbers can still lead to rather large errors.
		}[epsilon.Absolute(1e-5).Within(math.Sin(rtheta), 0)],
		rtheta,
	)

	return polar.Cartesian(
		polar.Add(
			*polar.New(0, a.heading.Theta()),
			rforce,
		),
	), a.accumulator.Force < a.limit.Force && a.accumulator.Torque < a.limit.Torque
}
