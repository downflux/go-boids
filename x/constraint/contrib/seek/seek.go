package seek

import (
	"github.com/downflux/go-boids/x/constraint/steer"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// SLSDO calculates a steering acceleration vector given an agent with a target
// position. This is the "classic" steering seek behavior as described in
// numerous sources.
//
// See https://slsdo.github.io/steering-behaviors/ for more information.
func SLSDO(a agent.RO) vector.V {
	return steer.Steer(
		func(a agent.RO) vector.V {
			buf := vector.M{0, 0}
			buf.Copy(a.TargetPosition())
			buf.Sub(a.Position())
			buf.Scale(a.MaxVelocity())
			return buf.V()
		},
	)(a)
}
