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
	// v is the relative velocity between the input agent and the incoming
	// obstacle. This is affords the agent some room for anticipatory
	// movement, which looks more natural.
	v := vector.Sub(a.V(), c.o.Obstacle.V())
	l := line.New(a.P(), v)

	// r is the minimum separation distance between the agent and the
	// obstacle. We need to factor in some uncertainty in how the agents
	// will move within the time period given. We are assuming the actual
	// acceleration is negligible.
	r := a.R() + c.o.Obstacle.R() + c.o.Tau*vector.Magnitude(v)

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

		if collision := t > 0; collision {
			avoid := vector.Sub(vector.Add(a.P(), v), c.o.Obstacle.P())
			// scalar represents how strong of a force the avoidance
			// vector is -- the shorter the avoidance vector, the
			// closer the agent is to the obstacle. Therefore, we
			// want to ensure that at the point of contact, the
			// agent is trying to steer away from the obstacle as
			// hard as possible.
			scalar := 1.0 / math.Max(epsilon, vector.Magnitude(avoid)-r)

			// We also need to scale the avoidance vector by the
			// actual relative velocity -- if the agent and obstacle
			// are moving in the same direction, then there is no
			// need to create some avoidance acceleration as they
			// will never actually collide.
			return vector.Scale(c.o.K*vector.Magnitude(v)*scalar, vector.Unit(avoid))
		}
	}
	return *vector.New(0, 0)
}
