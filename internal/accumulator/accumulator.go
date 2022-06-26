package accumulator

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

type A struct {
	limit       polar.V
	accumulator polar.V

	reference polar.V
}

func New(limit polar.V, v vector.V) *A {
	return &A{
		limit:       limit,
		accumulator: *polar.New(0, 0),
		reference:   polar.Normalize(polar.Polar(v)),
	}
}

// Add truncates the given input vector by the maximum change limits.  If the
// given input vector will exceed the given max, the limit bool return value
// will be set to false.
func (a *A) Add(v vector.V) (vector.V, bool) {
	p := polar.Normalize(polar.Polar(v))
	remainder := polar.Sub(a.limit, a.accumulator)

	// dtheta is the angular difference between the input vector and
	// reference. This angle may be negative.
	dtheta := p.Theta() - a.reference.Theta()

	// theta is the actual, absolute output angle.
	theta := a.reference.Theta() + math.Copysign(
		math.Min(remainder.Theta(), math.Abs(dtheta)),
		dtheta,
	)

	r := polar.Dot(
		*polar.New(1, p.Theta()),
		*polar.New(1, theta),
	) * math.Min(remainder.R(), p.R())

	q := *polar.New(r, theta)
	a.accumulator = polar.Add(
		a.accumulator,
		polar.Sub(
			polar.Normalize(q),
			*polar.New(0, a.reference.Theta()),
		),
	)

	return polar.Cartesian(q), polar.Within(p, q)
}
