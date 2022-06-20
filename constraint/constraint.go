package constraint

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// P is the priority of a given constraint. Note that higher priorities trend
// more negative. We are assigning float values to the underlying type of P, as
// future constraints may need to fall in between existing priorities, and
// floats are dense.
type P float64

func Less(p, q P) bool { return float64(q) < float64(p) }

type C interface {
	// A returns an acceleration vector for the input agent given the
	// constraint.
	A(a agent.A) vector.V

	// Priority returns the relative priority of the contraint --
	// higher priority constraints are processed first. Any lower-priority
	// constraints are dropped if the total acceleration returned exceeds
	// the acceleration threshold.
	Priority() P
}
