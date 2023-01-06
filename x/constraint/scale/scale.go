package scale

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Scale(f float64, c constraint.Accelerator) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		return vector.Scale(f, c(a))
	}
}
