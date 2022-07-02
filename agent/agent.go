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
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

type ID string

func (id ID) Equal(other ID) bool { return id == other }

type RW interface {
	RO
	WO
}

// WO is the locomotion layer of the simulation.
type WO interface {
	// Step takes as input a polar vector describing the movment and a
	// timestep. Note that this coordinate system allows for the agent to
	// turn while stationary.
	Step(steering vector.V, tau float64)
}

type RO interface {
	ID() ID

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

	MaxAcceleration() polar.V
}
