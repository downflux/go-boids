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
	strength = 10.0
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

// TODO(minkezhang): Implement the avoidance force as a sideways push tangent to
// the side of a sphere. See
// https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// for more details.
func (c C) A(a agent.A) vector.V {
	l := line.New(a.P(), a.V())
	r := a.R() + c.obstacle.R() + vector.Magnitude(a.V())

	lmin, lmax, ok := l.IntersectCircle(
		*hypersphere.New(c.obstacle.P(), r))
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
			avoid := vector.Sub(vector.Add(a.P(), a.V()), c.obstacle.P())
			return vector.Scale(strength, vector.Unit(avoid))
		}
	}
	return *vector.New(0, 0)
}
