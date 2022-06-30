package accumulator

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

type D struct {
	// Force is the total force accumulated during the time step, i.e. the
	// actual force magnitude (not just the radial component).
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

// d calculates the incremental force and torque of the given input force.
func (a *A) d(f vector.V) D {
	// theta is the relative angle directed from the heading to the force.
	theta := polar.Sub(polar.Polar(f), a.heading).Theta()
	return D{
		Force:  vector.Magnitude(f),
		Torque: a.radius * vector.Magnitude(f) * math.Sin(theta),
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

	delta := a.d(force)

	remainder := D{
		Force:  math.Max(0, a.limit.Force-a.accumulator.Force),
		Torque: math.Max(0, a.limit.Torque-a.accumulator.Torque),
	}

	df := D{
		Force: math.Min(math.Abs(delta.Force), remainder.Force),
		Torque: math.Copysign(
			math.Min(math.Abs(delta.Torque), remainder.Torque),
			delta.Torque,
		),
	}

	if epsilon.Relative(1e-5).Within(df.Force, 0) {
		return *vector.New(0, 0), true
	}

	// Preserve the input force angle, but shrink the output force vector to
	// ensure we meet the torque limit criteria.
	m := df.Force
	if !epsilon.Absolute(1e-5).Within(math.Remainder(theta, math.Pi), 0) {
		m = math.Min(df.Force, df.Torque/a.radius/math.Sin(theta))
	}

	truncated := vector.Scale(m, vector.Unit(force))
	d := a.d(truncated)

	a.accumulator = D{
		Force:  a.accumulator.Force + d.Force,
		Torque: a.accumulator.Torque + math.Abs(d.Torque),
	}

	return truncated, a.accumulator.Force < a.limit.Force && a.accumulator.Torque < a.limit.Torque
}
