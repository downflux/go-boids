package steering

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/accumulator"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
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
		//
		// TODO(minkezhang): Clamp the desired velocity to within the
		// absolute turning force.
		desired = vector.Scale(a.MaxVelocity().R(), vector.Unit(force))
	}

	// steering represents the total acceleration over the arbitrary time
	// scalar.
	steering := vector.Sub(desired, a.V())
	if vector.Within(steering, *vector.New(0, 0)) {
		return *vector.New(0, 0)
	}

	// We need to guarantee our steering acceleration does not exceed the
	// maximum allowable change for the next timestep.
	steering, _ = accumulator.New(
		*polar.New(
			tau*a.MaxAcceleration().R(),
			tau*a.MaxAcceleration().Theta(),
		),
		a.Heading(),
	).Add(steering)
	return steering
}
