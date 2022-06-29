package collision

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/constraint/weighted"
	"github.com/downflux/go-geometry/2d/vector"

	ca "github.com/downflux/go-boids/contrib/constraint/collision/agent"
)

var _ constraint.C = C{}

type C struct {
	o O
}

type O struct {
	Obstacles []agent.A
	K         float64
	Tau       float64
	MaxRange  float64
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

// TOOD(minkezhang): Toy with using a PQ here instead of a weighted average.
func (c C) Force(a agent.A) vector.V {
	var cs []constraint.C
	var ws []float64
	for _, o := range c.o.Obstacles {
		cs = append(cs, ca.New(ca.O{
			Obstacle: o,
			K:        c.o.K,
			Tau:      c.o.Tau,
			MaxRange: c.o.MaxRange,
		}))
		ws = append(ws, 1)
	}
	return weighted.New(cs, ws).Force(a)
}
