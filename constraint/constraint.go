package constraint

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

type C interface {
	// Force returns a force vector for the input agent given the constraint
	// parameters. Importantly, the force returned is independent of the
	// actual timestep tau.
	Force(a agent.A) vector.V
}
