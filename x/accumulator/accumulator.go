package accumulator

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

type A struct {
	limit       polar.V
	accumulator polar.V
	heading     polar.V
}

func New(limit polar.V, heading polar.V) *A {
	return &A{
		limit:       limit,
		accumulator: *polar.New(0, 0),
		heading:     heading,
	}
}

// Add truncates the given input vector by the maximum change limits.  If the
// given input vector will exceed the given max, the limit bool return value
// will be set to false.
func (a *A) Add(force vector.V) (vector.V, bool) {
	if vector.Within(force, *vector.New(0, 0)) {
		return *vector.New(0, 0), true
	}

	// f.Theta() here is the relative angle directed from the heading to the
	// force.
	f := *polar.New(
		polar.Polar(force).R(),
		math.Mod(
			polar.Polar(force).Theta()-a.heading.Theta(),
			2*math.Pi,
		),
	)

	// We know the torque on an object is defined as
	//
	//   T = || r x F || => ||T|| = rF * sin(𝜃)
	//   T = Iα
	//
	// Where F is the total (i.e. non-perpendicular) force applied, relative
	// to the agent heading. Note that critically, negative acceleration
	// does not overly-constrain the torque accumulator. We wish to emulate
	// this behavior, and not that the maximum amount of torque is applied
	// at right angle π/2 to the heading.

	if math.Mod(math.Abs(f.Theta()), 2*math.Pi) > math.Pi/2 {
		f = *polar.New(
			-f.R(),
			math.Mod(
				f.Theta()-math.Copysign(math.Pi, f.Theta()),
				2*math.Pi,
			),
		)
	}

	remainder := *polar.New(
		math.Max(0, a.limit.R()-a.accumulator.R()),
		math.Max(0, a.limit.Theta()-a.accumulator.Theta()),
	)

	f = *polar.New(
		math.Copysign(
			math.Min(
				math.Abs(f.R()),
				remainder.R(),
			),
			f.R(),
		),
		math.Copysign(
			math.Min(
				math.Abs(f.Theta()),
				remainder.Theta(),
			),
			f.Theta(),
		),
	)

	if epsilon.Absolute(1e-5).Within(math.Abs(f.R()), 0) {
		return *vector.New(0, 0), true
	}

	a.accumulator = *polar.New(
		a.accumulator.R()+math.Abs(f.R()),
		a.accumulator.Theta()+math.Abs(f.Theta()),
	)

	truncated := *polar.New(f.R(), f.Theta()+a.heading.Theta())

	return polar.Cartesian(truncated), a.accumulator.R() < a.limit.R() && a.accumulator.Theta() < a.limit.Theta()
}
