package arrival

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/steer"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// SLSDO calculates a steering acceleration vector given an agent with a target
// position. This acceleration vector brakes as the agent arrives at the
// goal.
//
// See https://slsdo.github.io/steering-behaviors/ for more information.
func SLSDO(v vector.V, r float64) constraint.Accelerator {
	return steer.Steer(
		func(a agent.RO) vector.V {
			buf := vector.M{0, 0}
			buf.Copy(v)
			buf.Sub(a.Position())
			e := a.MaxVelocity()
			if d := vector.Magnitude(buf.V()); d < r {
				// We are applying a stronger braking force to
				// ensure the agents do not oscillate.
				e *= d / (r * r)
			}
			buf.Scale(e)
			return buf.V()
		},
	)
}
