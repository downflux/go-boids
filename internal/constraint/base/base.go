package base

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C []constraint.C

func New(constraints []constraint.C) *C {
	c := C(constraints)
	return &c
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	// TODO(minkezhang): Account for a.MaxAcceleration().Theta() as well
	// here.
	v := *vector.New(0, 0)
	accumulator := *polar.New(0, 0)
	for _, constraint := range c {
		acceleration := constraint.A(a)
		// Ensure total acceleration influences is capped.
		if accumulator.R()+vector.Magnitude(acceleration) >= a.MaxAcceleration().R() {
			// Truncate "extra" acceleration influence.
			//
			// See Reynolds 1987 for more information on
			// acceleration truncation.
			acceleration = vector.Scale(
				a.MaxAcceleration().R()-accumulator.R(),
				vector.Unit(acceleration),
			)
		}

		accumulator = polar.Add(
			accumulator,
			*polar.New(vector.Magnitude(acceleration), 0),
		)
		v = vector.Add(v, acceleration)
		if accumulator.R() >= a.MaxAcceleration().R() {
			break
		}
	}

	if vector.Within(*vector.New(0, 0), v) {
		return *vector.New(0, 0)
	}

	return v
}
