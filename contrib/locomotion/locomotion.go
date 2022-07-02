package locomotion

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
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
	v := vector.Add(a.V(), steering)

	// w is the angular between the current agent heading and drected
	// towards the new velocity vector. Note that 0 <= w < 2π.
	w := polar.Normalize(
		polar.Sub(
			polar.Polar(v),
			a.Heading(),
		),
	).Theta()

	// Specify w such that the agent will turn in the optimal direction.
	// This ensures -π <= 0 < π.
	if w >= math.Pi {
		w -= 2 * math.Pi
	}

	// u is the net new velocity in polar coordinates rotated about the
	// agent reference frame.
	u := *polar.New(
		math.Min(a.MaxVelocity().R(), vector.Magnitude(v)),
		math.Copysign(
			math.Min(
				a.MaxVelocity().Theta(),
				math.Abs(w),
			),
			w,
		),
	)

	dw := w
	// In the case the angular velocity exceeds the maximal turnable rate,
	// model the move as a reverse step instead.
	//
	// If the turning velocity is greater than the absolute angular
	// velocity, and is also pointing away from the agent, model this
	// behavior as the agent reversing.
	if math.Abs(w) > math.Pi/2 && math.Abs(w)/tau > a.MaxVelocity().Theta() {
		// We should be rotating towards 0 -- this means our π offset
		// needs to ensure the new dw is still within [-π, π).
		dw = -math.Copysign(
			a.MaxVelocity().Theta(),
			dw,
		)
		u = *polar.New(-u.R(), dw)
	}

	dw = math.Copysign(
		math.Min(
			a.MaxVelocity().Theta(),
			math.Abs(dw),
		),
		dw,
	)

	b := mock.New(mock.O{
		V: polar.Cartesian(
			*polar.New(
				u.R(),
				a.Heading().Theta()+tau*u.Theta(),
			),
		),
		Heading: *polar.New(1, a.Heading().Theta()+tau*dw),
	})
	b.O.P = vector.Add(a.P(), vector.Scale(tau, b.V()))

	return b
}
