package collision

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
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
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	v := *vector.New(0, 0)
	for _, o := range c.o.Obstacles {
		v = vector.Add(v, vector.Scale(1.0/float64(len(c.o.Obstacles)),
			ca.New(ca.O{
				Obstacle: o,
				K:        c.o.K,
				Tau:      c.o.Tau,
			}).A(a)))
	}
	return v
}
