package agent

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type O struct {
	Neighbor agent.RO
}

type C struct {
	o O
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

func (c C) Accelerate(a agent.RO) vector.V { return c.o.Neighbor.V() }
