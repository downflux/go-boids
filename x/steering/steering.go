package steering

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// S returns the desired velocity for the next tick for a given agent with the
// calculated input Boid force.
func S(a agent.RO, force vector.V, tau float64) vector.V {
	desired := *vector.New(0, 0)
	if !vector.Within(force, *vector.New(0, 0)) {
		// Agent's locomotion directive is to travel as fast as possible
		// in the direction indicated by the force vector.
		//
		// TODO(minkezhang): Ensure MaxVelocity is calculated
		// dynamically.
		desired = vector.Scale(a.MaxVelocity().R(), vector.Unit(force))
	}

	if vector.Within(desired, a.V()) {
		return *vector.New(0, 0)
	}

	return vector.Scale(
		math.Min(
			tau*a.MaxNetForce()/a.Mass(),
			vector.Magnitude(vector.Sub(desired, a.V())),
		),
		vector.Unit(vector.Sub(desired, a.V())),
	)
}
