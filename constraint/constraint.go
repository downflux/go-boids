package constraint

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

type C interface {
	// Boid returns a force vector for the input agent given the constraint.
	Boid(a agent.A) vector.V
}
