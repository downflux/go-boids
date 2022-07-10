package base

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-boids/x/constraint/clamped"
	"github.com/downflux/go-boids/x/constraint/contrib/collision"
	"github.com/downflux/go-boids/x/kd"
	"github.com/downflux/go-geometry/2d/vector"
)

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

func (c C) Steer(a agent.RO) vector.V {
	return clamped.New([]constraint.Steer{
		collision.New(collision.O{
			T:      c.o.T,
			K:      c.o.CollisionWeight,
			Cutoff: a.MaxSpeed() + 5*c.o.R,
			Filter: c.o.CollisionFilter,
		}).Steer,
	}).Steer(a)
}