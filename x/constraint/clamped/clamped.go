package clamped

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
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

func (c C) Accelerate(a agent.RO) vector.V {
	acc := 0.0

	n := *vector.New(0, 0)

	for _, d := range c {
		n = vector.Add(
			n, agent.Clamp(d.Accelerate(a), 0, a.MaxNetAcceleration()-acc),
		)
	}
	return n
}
