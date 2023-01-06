package avoidance

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/vector"
)

// SLSDO calculates a steering acceleration given an agent-agent interaction.
// This is based on the slsdo avoidance algorithm. See
// https://slsdo.github.io/steering-behaviors/ for more information.
//
// N.B.: In the original slsdo algorithm, there is consideration for the
// time-to-collision (lb). We assume this value is accounted for by the caller,
// which should iterate over all local neighbors of the input agent a.
func SLSDO(a agent.RO, o agent.RO) vector.V {
	am, om := a.Mass(), o.Mass()
	ap, av := a.Position(), a.Velocity()
	op := o.Position()

	l := line.New(ap, av)

	buf := vector.M{0, 0}

	// Check that we are traveling in the direction of the opposing agent.
	if t := l.T(op); t > 0 {
		// Check if we will collide with the opposing agent. Add a small
		// buffer to the detection, as this only detects perpendicular
		// hits.
		r := 1.5 * (a.Radius() + o.Radius())
		if d := l.Distance(op); d < r {
			// Calculate the projected distance vector between the
			// agent at passthrough point and the obstacle.
			//
			// TODO(minkezhang): Handle direct oncoming collisions.
			buf.Copy(op)
			buf.Sub(l.L(t))
			buf.Unit()
			e := (r - d) / r * a.MaxVelocity()
			e *= om / am
			buf.Scale(-e)
		}
	}

	return buf.V()
}
