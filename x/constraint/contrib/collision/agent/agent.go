package agent

import (
	"math"
	"math/rand"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

var _ constraint.C = C{}

type C struct {
	o O
}

type O struct {
	K        float64
	Obstacle agent.RO
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
func (c C) Accelerate(a agent.RO) vector.V {
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

	a.Logger().Printf("DEBUG(agent.Steer): separation = %.3f, d = %.3f", separation, vector.Magnitude(p))

	return vector.Scale(c.o.K/separation, vector.Unit(p))
}
