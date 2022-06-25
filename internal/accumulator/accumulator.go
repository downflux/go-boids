package accumulator

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

type A struct {
	limit       polar.V
	accumulator polar.V

	theta       float64
	initialized bool
}

func New(limit polar.V) *A {
	return &A{
		limit:       limit,
		accumulator: *polar.New(0, 0),
	}
}

// Add truncates the given input acceleration by the maximum acceleration
// limits. If the given acceleration will exceed the given max, the limit bool
// return value will be set to false.
func (a *A) Add(acceleration vector.V) (vector.V, bool) {
	p := polar.Polar(acceleration)
	if !a.initialized {
		a.initialized = true
		a.theta = p.Theta()
	}

	remainder := *polar.New(
		a.limit.R()-a.accumulator.R(),
		a.limit.Theta()-math.Abs(a.accumulator.Theta()-a.theta),
	)
	q := *polar.New(
		math.Min(remainder.R(), p.R()),
		math.Min(remainder.Theta(), p.Theta()))
	a.accumulator = polar.Add(a.accumulator, q)

	a.theta = q.Theta()

	return polar.Cartesian(q), polar.Within(p, q)
}
