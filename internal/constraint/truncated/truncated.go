package truncated

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/accumulator"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C []constraint.C

// TODO(minkezhang): Add priority list here as well in case the constraints are
// not ordered.
func New(constraints []constraint.C) *C {
	c := C(constraints)
	return &c
}

func (c C) Force(a agent.RO) vector.V {
	acc := accumulator.New(a.MaxAcceleration(), a.Heading())

	f := *vector.New(0, 0)

	for _, constraint := range c {
		g, ok := acc.Add(vector.Scale(1/a.Mass(), constraint.Force(a)))
		f = vector.Add(f, g)
		if !ok {
			break
		}
	}

	return vector.Scale(a.Mass(), f)
}
