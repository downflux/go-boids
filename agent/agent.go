// Package agent provides an interface for a non-holonomic robot used for the
// Boid simulation.
//
// We are modeling this robot with a maximum speed and angular velocity, which
// means the robot will have a non-zero turning radius. This should lead to
// smoother movement over naive disks, which tend to have very "jerky" behavior
// due to the lack of limit on angular velocity.
//
// We model the agent as a uniform disk which is turning about its center of
// mass -- this means its moment of intertia I can be defined as
//
//   I = 1/2 mr ** 2
//
// Note that we could also trivially redefine the "wheels" of the agent to be
// pivoting from the edge of the robot, i.e.
//
//   I = 1/3 mr ** 3, where r is the radius of the agent
//
// In this case however, the locomotion layer must take into account that
// movement will stem from the wheel well (i.e. the edge of the circle with the
// given heading) and calculate the center of mass accordingly.
package agent

import (
	"math"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

type RW interface {
	RO
	WO
}

// WO is the locomotion layer of the simulation.
type WO interface {
	// Step takes as input a polar vector describing the movment and a
	// timestep. Note that this coordinate system allows for the agent to
	// turn while stationary.
	Locomotion(steering vector.V, tau float64)
}

type RO interface {
	P() vector.V
	V() vector.V
	R() float64
	Mass() float64
	Goal() vector.V

	// Heading is a unit polar vector oriented towards the current direction
	// the agent is facing. Note that by necessity, the angular component is
	// relative to the global axis.
	Heading() polar.V

	// MaxVelocity constraints the agent with a maximum speed and angular
	// velocity.
	MaxVelocity() polar.V

	// MaxNetForce is the total radial force that an agent can experience
	// for a single step. Here, the radial component is aligned with the
	// agent heading.
	//
	// We know that
	//
	//   F = ma
	//
	// Therefore, we can limit the maximum radial acceleration via this
	// constant.
	MaxNetForce() float64

	// MaxNetTorque is the total torque an agent can experience for a single
	// step. Note that
	//
	//   T = r x F => ||T|| = rF * sin(𝜃)
	//   T = Iα
	//
	// Where 𝜃 is the angle between the agent heading and the force.
	//
	// This is the rotational analogue to the MaxNetForce, and can be used
	// to limit the angular acceleration α (similar to MaxNetForce limiting
	// the net radial acceleration ||a||.
	MaxNetTorque() float64
}

// Steer returns the desired velocity for the next tick for a given agent with
// the calculated input Boid force.
func Steer(a RO, force vector.V, tau float64) vector.V {
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
