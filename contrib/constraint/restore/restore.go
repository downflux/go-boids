package restore

import (
	// "math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type O struct {
	K float64
}

type C struct {
	o O
}

func New(o O) *C { return &C{o: o} }

func (c C) Force(a agent.RO) vector.V {
	// In the case that an agent is moving backwards, we want to apply a
	// consistent turning force to force the agent to move in the same
	// direction as its heading.
	if vector.Dot(polar.Cartesian(a.Heading()), a.V()) < 0 {
		return vector.Scale(a.MaxAcceleration().R()*a.Mass(), polar.Cartesian(
			polar.Add(
				a.Heading(),
				*polar.New(0, 0), // math.Pi),
			),
		))
	}
	return *vector.New(0, 0)
}
