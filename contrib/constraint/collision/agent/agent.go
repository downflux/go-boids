package agent

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

const (
	epsilon = 1e-3
)

var _ constraint.C = C{}

type C struct {
	obstacle agent.A
	tau      float64
}

type O struct {
	Obstacle agent.A
	Tau      float64
}

func New(o O) *C {
	return &C{
		obstacle: o.Obstacle,
		tau:      o.Tau,
	}
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	r := vector.Scale(1, vector.Sub(a.P(), c.obstacle.P()))
	d := math.Max(0, vector.Magnitude(r)-(c.obstacle.R()+a.R()))
	cutoff := 10 * (c.obstacle.R() + a.R())
	if d > cutoff {
		return *vector.New(0, 0)
	}

	return vector.Scale(c.tau/math.Max(epsilon, d*d), vector.Unit(r))
}
