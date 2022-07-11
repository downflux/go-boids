package base

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-boids/x/constraint/clamped"
	"github.com/downflux/go-boids/x/constraint/contrib/arrival"
	"github.com/downflux/go-boids/x/constraint/contrib/collision"
	"github.com/downflux/go-boids/x/kd"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type O struct {
	T   *kd.T
	R   float64
	Tau float64

	CollisionWeight float64
	CollisionFilter func(a agent.RO) bool

	ArrivalWeight float64
}

type C struct {
	o O
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

func (c C) Accelerate(a agent.RO) vector.V {
	cs := []constraint.C{
		collision.New(collision.O{
			T:      c.o.T,
			K:      1, // c.o.CollisionWeight,
			Cutoff: c.o.Tau*a.MaxSpeed() + 5*c.o.R,
			Filter: c.o.CollisionFilter,
		}),
		arrival.New(arrival.O{
			K: 1, // c.o.ArrivalWeight,
		}),
	}
	a.Logger().Printf("DEBUG(base.Accelerate): collision force: %v", vector.Magnitude(cs[0].Accelerate(a))*2000)
	a.Logger().Printf("DEBUG(base.Accelerate): arrival force: %v", vector.Magnitude(cs[1].Accelerate(a))*2000)

	cs = []constraint.C{
		constraint.Steer(cs[0], c.o.CollisionWeight),
		constraint.Steer(cs[1], c.o.ArrivalWeight),
	}
	a.Logger().Printf("DEBUG(base.Accelerate): [steering] collision force: %v", vector.Magnitude(cs[0].Accelerate(a))*2000)
	a.Logger().Printf("DEBUG(base.Accelerate): [steering] arrival force: %v", vector.Magnitude(cs[1].Accelerate(a))*2000)

	b := clamped.New(cs).Accelerate(a)
	a.Logger().Printf("DEBUG(base.Accelerate): [steering] clamped force: %v", vector.Magnitude(b)*2000)

	return b
}
