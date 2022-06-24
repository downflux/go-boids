package agent

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/vector"
)

const (
	epsilon = 1e-1
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

func (c C) Priority() constraint.P { return 0 }

// TODO(minkezhang): Implement the avoidance force as a sideways push tangent to
// the side of a sphere. See
// https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// for more details.
func (c C) A(a agent.A) vector.V {
	l := line.New(a.P(), a.V())
	r := a.R() + c.o.Obstacle.R() + c.o.Tau*vector.Magnitude(a.V())

	lmin, lmax, ok := l.IntersectCircle(
		*hypersphere.New(c.o.Obstacle.P(), r))
	// The obstacle ray-traces an intersection with the obstacle. This may
	// have happend in the past though, so we need to check if the
	// intersection is in the positive direction.
	if ok {
		tmin := l.T(lmin)
		tmax := l.T(lmax)

		t := math.Inf(-1)
		if tmin > 0 {
			t = tmin
		} else if tmax > 0 {
			t = tmax
		}

		if t > 0 {
			avoid := vector.Sub(vector.Add(a.P(), a.V()), c.o.Obstacle.P())
			scalar := math.Max(epsilon, vector.Magnitude(avoid)-2*r)
			return vector.Scale(c.o.K/scalar, vector.Unit(avoid))
		}
	}
	return *vector.New(0, 0)
}
