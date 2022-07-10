package constraint

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

type C interface {
	// Accelerate is a function which returns the desired net acceleration
	// for a specific set of forces. For seek behavior, the acceleration
	// vector will point directly to the target.
	Accelerate(a agent.RO) vector.V
}

type steer func(a agent.RO) vector.V
func (s steer) Accelerate(a agent.RO) vector.V {
	return agent.Steer(a, s(a), 1)
}

func Steer(c C) C {
	return steer(c.Accelerate)
}
