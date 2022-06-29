package agent

import (
	"math"
	"math/rand"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

var _ constraint.C = C{}

type C struct {
	o O
}

type O struct {
	Obstacle agent.A
	K        float64
	Tau      float64
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

func jitter() vector.V {
	return vector.Unit(
		*vector.New(
			rand.Float64()+1e-5,
			rand.Float64()+1e-5,
		),
	)
}

// TODO(minkezhang): Implement the avoidance force as a sideways push tangent to
// the side of a sphere. See
// https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// for more details.
func (c C) Force(a agent.A) vector.V {
	p := vector.Sub(a.P(), c.o.Obstacle.P())
	if vector.Within(p, *vector.New(0, 0)) {
		p = jitter()
	}
	if epsilon.Within(
		math.Remainder(c.o.Obstacle.Heading().Theta()-a.Heading().Theta(), math.Pi),
		0,
	) && epsilon.Within(
		vector.Determinant(polar.Cartesian(a.Heading()), p),
		0,
	) {
		p = vector.Add(p, vector.Scale(0.1+vector.Magnitude(p), vector.Unit(jitter())))
	}

	r := c.o.Obstacle.R() + a.R()
	separation := math.Max(
		1e-5, vector.SquaredMagnitude(p)/r/r)

	return vector.Scale(c.o.K/separation*a.Mass(), vector.Unit(p))
}
