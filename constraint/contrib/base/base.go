package base

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/clamped"
	"github.com/downflux/go-boids/constraint/contrib/alignment"
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

	AlignmentWeight float64
	AlignmentFilter func(a agent.RO) bool
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
	return clamped.New([]constraint.C{
		// TODO(minkezhang): Set the acceleration directly as a
		// constraint here to re-inforce the near-hard sphere collision.
		// The radius of influence for this constraint should be
		// basically a delta function. The collision avoidance potential
		// is a separate smoothing function layered on top of the actual
		// collision constraint.
		constraint.Steer(
			collision.New(collision.O{
				T:      c.o.T,
				Cutoff: 7 * c.o.R,
				Filter: c.o.CollisionFilter,
			}),
			c.o.CollisionWeight,
			math.Inf(0),
		),
		constraint.Steer(
			arrival.New(arrival.O{}),
			c.o.ArrivalWeight,
			a.MaxNetAcceleration(),
		),
		constraint.Steer(
			alignment.New(alignment.O{
				T:      c.o.T,
				Cutoff: 10 * c.o.R,
				Filter: c.o.AlignmentFilter,
			}),
			c.o.AlignmentWeight,
			a.MaxNetAcceleration(),
		),
	}).Accelerate(a)
}
