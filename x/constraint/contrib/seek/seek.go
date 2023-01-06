package seek

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// SLSDO calculates a steering acceleration vector given an agent with a target
// position.
//
// See https://slsdo.github.io/steering-behaviors/ for more information.
func SLSDO(a agent.RO) vector.V {
	buf := vector.M{0, 0}
	buf.Copy(a.TargetPosition())
	buf.Sub(a.Position())
	if d := vector.Magnitude(buf.V()); d > 0 {
		buf.Unit()
		buf.Scale(a.MaxVelocity())
		buf.Sub(a.Velocity())
	}
	return buf.V()
}
