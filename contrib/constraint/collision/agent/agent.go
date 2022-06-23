package agent

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

const (
	epsilon  = 1e-3
	strength = 5.0
)

var _ constraint.C = C{}

type C struct {
	obstacle agent.A
}

type O struct {
	Obstacle agent.A
}

func New(o O) *C {
	return &C{
		obstacle: o.Obstacle,
	}
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	r := vector.Sub(a.P(), c.obstacle.P())
	d := math.Max(0, vector.Magnitude(r)-(c.obstacle.R()+a.R()))
	cutoff := 5 * (c.obstacle.R() + a.R())
	if d > cutoff {
		return *vector.New(0, 0)
	}

	return vector.Scale(strength/math.Max(epsilon, d*d), vector.Unit(r))
}
