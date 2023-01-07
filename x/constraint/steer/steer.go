package steer

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Steer(c constraint.Accelerator) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		buf := vector.M{0, 0}
		buf.Copy(c(a))
		// c(a) returns an acceleration vector over the next second. We
		// assume t = 1 (since the agent velocity is also over the next
		// second).
		if d := vector.Magnitude(buf.V()); d > 0 {
			buf.Unit()
			buf.Scale(a.MaxAcceleration())
			buf.Sub(a.Velocity())
		}
		return buf.V()
	}
}
