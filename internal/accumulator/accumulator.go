package accumulator

import (
	"math"

	"github.com/downflux/go-geometry/2d/vector"
)

const (
	epsilon = 1e-5
)

type A struct {
	limit       float64
	accumulator float64
}

func New(limit float64) *A {
	return &A{
		limit: limit,
	}
}

// Add truncates the given input vector by the maximum change limits.  If the
// given input vector will exceed the given max, the limit bool return value
// will be set to false.
func (a *A) Add(v vector.V) (vector.V, bool) {
	if vector.Within(v, *vector.New(0, 0)) {
		return *vector.New(0, 0), true
	}

	v = vector.Scale(
		math.Min(
			math.Max(
				0,
				a.limit-a.accumulator,
			),
			vector.Magnitude(v),
		),
		vector.Unit(v),
	)
	a.accumulator += vector.Magnitude(v)
	return v, a.accumulator < a.limit-epsilon
}
