package locomotion

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-boids/x/accumulator"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-boids/agent/mock"
)

// L advances the Boid simulation by a single step.
//
// The agent will reverse if the velocity is negative and the agent cannot turn
// in the time allotted.
//
// The input steering vector is the change in velocity from the current velocity
// of the agent; the ideal outcome is for the agent to set its own velocity to
// the vector sum of these two values, but because of physical constraints on
// the agent (i.e.  maximum speed and angular velocity), we must clamp the
// desired velocity.
//
// The output is a pseudo-agent whose properties may be used by the caller to
// directly mutate the calling agent, i.e.
//
//   b := L(a, steering, tau)
//   a.v = b.V()
//   a.p = b.P()
//   a.heading = b.Heading()
//
// See Reynolds (1999) for more information or Sebastion Lague's Boids project
// for reference (https://github.com/SebLague/Boids).
func L(a agent.RO, steering vector.V, tau float64) agent.RO {
	v, _ := accumulator.New(a.MaxVelocity(), a.Heading()).Add(
		vector.Add(a.V(), steering),
	)

	if vector.Within(v, *vector.New(0, 0)) {
		return mock.New(mock.O{
			ID:      a.ID(),
			P:       a.P(),
			V:       *vector.New(0, 0),
			Heading: a.Heading(),
		})
	}

	return mock.New(mock.O{
		ID: a.ID(),
		P:  vector.Add(a.P(), vector.Scale(tau, v)),
		V:  v,
		Heading: map[bool]polar.V{
			true: *polar.New(
				1,
				polar.Polar(v).Theta(),
			),
			false: *polar.New(
				1,
				math.Mod(polar.Polar(v).Theta()+math.Pi, 2*math.Pi),
			),
		}[vector.Dot(
			polar.Cartesian(a.Heading()), v) > 0],
	})
}
