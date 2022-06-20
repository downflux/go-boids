package agent

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/hypersphere"
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

func (c C) Boid(a agent.A) vector.V {
	distance := vector.Magnitude(vector.Scale(c.tau, c.obstacle.V())) + 0.5*c.tau*c.tau*c.obstacle.MaxAcceleration().R()
	avoidance := hypersphere.New(
		c.obstacle.P(),
		c.obstacle.R()+distance)
	cutoff := avoidance.R() + 2*a.R()
	d := vector.Magnitude(vector.Sub(a.P(), c.obstacle.P()))
	if d > cutoff {
		return *vector.New(0, 0)
	}
	return vector.Scale(
		a.M()/math.Max(epsilon, d*d),
		vector.Unit(vector.Sub(a.P(), c.obstacle.P())),
	)
}
