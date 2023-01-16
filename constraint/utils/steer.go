package utils

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

// Steer takes as input a desired velocity and returns a steering acceleration
// vector.
func Steer(v func(agent.RO) vector.V) constraint.Steer {
	return func(a agent.RO) vector.V {
		// v(a) returns a desired velocity vector.  We assume t = 1
		// (since the agent velocity is also over the next second).
		desired := v(a)

		buf := vector.M{0, 0}
		buf.Copy(desired)
		buf.Sub(a.Velocity())

		// Check for stop condition. If the desired velocity is small
		// (i.e. stop), and the current velocity is close to the desired
		// velocity, force the agent steering vector to oppose the
		// current velocity. This smooths out some oscillatory behavior
		// at low speeds.
		if d := vector.Magnitude(buf.V()); epsilon.Absolute(1e-5).Within(vector.Magnitude(desired), 0) && d < a.MaxAcceleration() {
			buf.Copy(a.Velocity())
			buf.Scale(-1)
		} else if d > 0 {
			buf.Unit()
			buf.Scale(a.MaxAcceleration())
		}
		return buf.V()
	}
}
