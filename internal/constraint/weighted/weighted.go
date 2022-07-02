package weighted

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C struct {
	constraints []constraint.C
	weights     []float64
}

func New(constraints []constraint.C, weights []float64) *C {
	if len(constraints) != len(weights) {
		panic("mismatched array lengths for constraints and weights")
	}
	return &C{
		constraints: constraints,
		weights:     weights,
	}
}

func (c C) Force(a agent.RO) vector.V {
	if len(c.constraints) == 0 {
		return *vector.New(0, 0)
	}

	parts := 0.0
	for _, w := range c.weights {
		parts += w
	}

	v := *vector.New(0, 0)
	for i, constraint := range c.constraints {
		f := constraint.Force(a)
		v = vector.Add(v, vector.Scale(c.weights[i]/parts, f))
	}

	if vector.Within(v, *vector.New(0, 0)) {
		return *vector.New(0, 0)
	}
	return vector.Scale(
		math.Min(
			a.Mass()*a.MaxAcceleration().R(),
			vector.Magnitude(v),
		), vector.Unit(v))
}
