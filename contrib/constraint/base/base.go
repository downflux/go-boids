package base

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/contrib/constraint/arrival"
	"github.com/downflux/go-boids/contrib/constraint/collision"
	"github.com/downflux/go-boids/internal/constraint/truncated"
	"github.com/downflux/go-boids/kd"
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

func (c C) Force(a agent.RO) vector.V {
	return truncated.New([]constraint.C{
		collision.New(collision.O{
			T:      c.o.T,
			K:      c.o.CollisionWeight,
			Cutoff: c.o.Tau*a.MaxVelocity().R() + 5*c.o.R,
			Filter: c.o.CollisionFilter,
		}),
		arrival.New(arrival.O{
			K: c.o.ArrivalWeight,
		}),
	}).Force(a)
}
