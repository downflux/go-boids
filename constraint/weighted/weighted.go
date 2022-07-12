package weighted

import (
	"fmt"
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

var _ constraint.C = C{}

type C struct {
	cs []constraint.C
	ws []float64
}

func New(cs []constraint.C, ws []float64) *C {
	if len(cs) != len(ws) {
		panic(fmt.Sprintf("mismatching constraint and weight lengths: %v != %v", len(cs), len(ws)))
	}
	return &C{
		cs: cs,
		ws: ws,
	}
}

func (c C) Name() string { return "" }

func (c C) Accelerate(a agent.RO) vector.V {
	if len(c.cs) == 0 {
		return *vector.New(0, 0)
	}

	parts := 0.0
	for _, w := range c.ws {
		parts += w
	}

	n := *vector.New(0, 0)
	for i, d := range c.cs {
		n = vector.Add(n, vector.Scale(c.ws[i]/parts, d.Accelerate(a)))
	}
	if epsilon.Within(vector.SquaredMagnitude(n), 0) {
		return *vector.New(0, 0)
	}

	return vector.Scale(math.Min(a.MaxNetAcceleration(), vector.Magnitude(n)), vector.Unit(n))
}
