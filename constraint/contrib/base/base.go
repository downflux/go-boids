package base

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/clamped"
	"github.com/downflux/go-boids/constraint/contrib/arrival"
	"github.com/downflux/go-boids/constraint/contrib/collision"
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

func (c C) Accelerate(a agent.RO) vector.V {
	// TODO(minkezhang): Ensure the steering force can be sufficiently
	// weighted such that the collision force can exceed the maximum
	// acceleration threshold.
	return clamped.New([]constraint.C{
		constraint.Steer(
			collision.New(collision.O{
				T:      c.o.T,
				Cutoff: c.o.Tau*a.MaxSpeed() + 5*c.o.R,
				Filter: c.o.CollisionFilter,
			}),
			c.o.CollisionWeight,
		),
		constraint.Steer(
			arrival.New(arrival.O{}),
			c.o.ArrivalWeight,
		),
	}).Accelerate(a)
}
