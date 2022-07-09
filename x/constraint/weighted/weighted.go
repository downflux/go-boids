package weighted

import (
	"math"

	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

type C struct {
	cs []constraint.Steer
	ws []float64
}

func (c C) Steer(a agent.RO) vector.V {
	if len(c.cs) == 0 {
		return *vector.New(0, 0)
	}

	parts := 0.0
	for _, w := range c.ws {
		parts += w
	}

	n := *vector.New(0, 0)
	for i, d := range c.cs {
		n = vector.Add(n, vector.Scale(c.ws[i]/parts, d(a)))
	}
	if epsilon.Within(vector.SquaredMagnitude(n), 0) {
		return *vector.New(0, 0)
	}

	return vector.Scale(math.Min(a.MaxNetAcceleration(), vector.Magnitude(n)), vector.Unit(n))
}
