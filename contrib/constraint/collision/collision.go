package collision

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"

	ca "github.com/downflux/go-boids/contrib/constraint/collision/agent"
)

var _ constraint.C = C{}

type C struct {
	obstacles []agent.A
	tau       float64
}

type O struct {
	Obstacles []agent.A
	Tau       float64
}

func New(o O) *C {
	return &C{
		obstacles: o.Obstacles,
		tau:       o.Tau,
	}
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	v := *vector.New(0, 0)
	for _, o := range c.obstacles {
		v = vector.Add(v, vector.Scale(1.0/float64(len(c.obstacles)),
			ca.New(ca.O{
				Obstacle: o,
				Tau:      c.tau,
			}).A(a)))
	}
	return v
}
