package agent

import (
	"fmt"
	"math"
	"math/rand"
	"os"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
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
	v := vector.Sub(c.o.Obstacle.V(), a.V())
	p := vector.Sub(c.o.Obstacle.P(), a.P())

	// r is the effective radius of avoidance
	r := c.o.Obstacle.R() + a.R() + vector.Magnitude(v)

	ahead := vector.Add(a.P(), a.V())
	approach := math.Min(
		vector.Magnitude(p),
		math.Min(
			vector.Magnitude(vector.Sub(ahead, c.o.Obstacle.P())),
			vector.Magnitude(
				vector.Sub(
					vector.Add(a.P(), vector.Scale(0.5, a.V())),
					c.o.Obstacle.P(),
				),
			),
		),
	)
	if approach <= r {
		f := vector.Sub(ahead, c.o.Obstacle.P())
		if vector.Within(f, *vector.New(0, 0)) {
			return *vector.New(0, 0)
		}
		separation := math.Max(1e-5, approach)
		fmt.Fprintf(os.Stderr, "DEBUG(agent.Force): f: %v, separation: %v\n", f, separation)

		return vector.Scale(
			c.o.K*a.Mass()/separation, vector.Unit(f),
		)

	}

	return *vector.New(0, 0)
}
