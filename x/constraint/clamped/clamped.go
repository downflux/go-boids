package clamp

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

type C []constraint.Steer

// TODO(minkezhang): Add priority list here as well in case the constraints are
// not ordered.
func New(constraints []constraint.Steer) *C {
	c := C(constraints)
	return &c
}

func (c C) Steer(a agent.RO) vector.V {
	acc := 0.0

	n := *vector.New(0, 0)

	for _, d := range c {
		n = vector.Add(
			n, agent.Clamp(d(a), 0, a.MaxNetAcceleration()-acc),
		)
	}
	return n
}
