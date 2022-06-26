package accumulator

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

type A struct {
	limit       polar.V
	accumulator polar.V

	velocity polar.V
}

func New(limit polar.V, v vector.V) *A {
	return &A{
		limit:       limit,
		accumulator: *polar.New(0, 0),
		velocity:    polar.Normalize(polar.Polar(v)),
	}
}

// Add truncates the given input acceleration by the maximum acceleration
// limits. If the given acceleration will exceed the given max, the limit bool
// return value will be set to false.
func (a *A) Add(acceleration vector.V) (vector.V, bool) {
	p := polar.Normalize(polar.Polar(acceleration))
	remainder := polar.Sub(a.limit, a.accumulator)

	// dtheta is the angular difference between the acceleration and
	// velocity. This angle may be negative.
	dtheta := p.Theta() - a.velocity.Theta()

	// theta is the actual, absolute accleration angle.
	theta := a.velocity.Theta() + math.Copysign(
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
			*polar.New(0, a.velocity.Theta()),
		),
	)

	return polar.Cartesian(q), polar.Within(p, q)
}
