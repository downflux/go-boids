package steer

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

func Steer(c constraint.Accelerator) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		v := c(a)
		if epsilon.Absolute(1e-5).Within(vector.SquaredMagnitude(v), 0) {
			return v
		}
		// c(a) returns an acceleration vector over the next second. We
		// assume t = 1 (since the agent velocity is also over the next
		// second).
		return vector.Sub(v, a.Velocity())
	}
}
