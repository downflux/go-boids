package config

import (
	"encoding/json"
	"fmt"
	"math"
	"os"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.RW = &A{}

type O struct {
	P            vector.V
	V            vector.V
	R            float64
	Goal         vector.V
	Heading      polar.V
	Mass         float64
	MaxNetForce  float64
	MaxNetTorque float64
	MaxVelocity  polar.V
}

type A struct {
	O
}

func (a *A) P() vector.V           { return a.O.P }
func (a *A) V() vector.V           { return a.O.V }
func (a *A) R() float64            { return a.O.R }
func (a *A) Goal() vector.V        { return a.O.Goal }
func (a *A) Heading() polar.V      { return a.O.Heading }
func (a *A) Mass() float64         { return a.O.Mass }
func (a *A) MaxVelocity() polar.V  { return a.O.MaxVelocity }
func (a *A) MaxNetTorque() float64 { return a.O.MaxNetTorque }
func (a *A) MaxNetForce() float64  { return a.O.MaxNetForce }

// Step advances the Boid simulation by a single step.
//
// The implemented agent will turn in place if the magnitude of the velocity is
// zero.
//
// We could alteratively instruct the agent to reverse if the angle is too
// large.
//
// The input velocity is the incremental velocity, and will need to be added to
// the current one.
// The angular component of the input vector is relative to the agent heading.
func (a *A) Step(steering vector.V, tau float64) {
	data, _ := json.MarshalIndent(
		map[string]string{
			"steering":    fmt.Sprintf("%.3f, %.3f)", steering.X(), steering.Y()),
			"a.P()":       fmt.Sprintf("%.3f, %.3f)", a.P().X(), a.P().Y()),
			"a.Heading()": fmt.Sprintf("%.3f, %.3f)", a.Heading().R(), a.Heading().Theta()),
			"a.V()":       fmt.Sprintf("%.3f, %.3f)", a.V().X(), a.V().Y()),
		},
		"", "  ")
	fmt.Fprintf(os.Stderr, "DEBUG(config.Step): before %s\n", data)

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
	data, _ = json.MarshalIndent(
		map[string]string{
			"w": fmt.Sprintf("%.3f", w),
		},
		"", "  ")
	fmt.Fprintf(os.Stderr, "DEBUG(config.Step): w %s\n", data)

	dw := w
	// In the case the angular velocity exceeds the maximal turnable rate,
	// model the move as a reverse step instead.
	//
	// If the turning velocity is greater than the absolute angular
	// velocity, and is also pointing away from the agent, model this
	// behavior as the agent reversing.
	if math.Abs(w) > math.Pi/2 && math.Abs(w)/tau > a.MaxVelocity().Theta() {
		fmt.Fprintf(os.Stderr, "DEBUG(config.Step): need to flip heading and velocity\n")
		// We should be rotating towards 0 -- this means our π offset
		// needs to ensure the new dw is still within [-π, π).
		dw = -math.Copysign(
			a.MaxVelocity().Theta(),
			dw,
		)
		// u = *polar.New(-u.R(), dw)
		// u = *polar.New(u.R(), dw)
		u = *polar.New(-u.R(), dw)
	}

	dw = math.Copysign(
		math.Min(
			a.MaxVelocity().Theta(),
			math.Abs(dw),
		),
		dw,
	)

	a.O.V = polar.Cartesian(
		*polar.New(
			u.R(),
			a.Heading().Theta()+tau*u.Theta(),
		),
	)
	a.O.Heading = *polar.New(1, a.Heading().Theta()+tau*dw)
	a.O.P = vector.Add(a.P(), vector.Scale(tau, a.V()))

	data, _ = json.MarshalIndent(
		map[string]string{
			"a.P()":       fmt.Sprintf("%.3f, %.3f)", a.P().X(), a.P().Y()),
			"a.Heading()": fmt.Sprintf("%.3f, %.3f)", a.Heading().R(), a.Heading().Theta()),
			"a.V()":       fmt.Sprintf("%.3f, %.3f)", a.V().X(), a.V().Y()),
		},
		"", "  ")
	fmt.Fprintf(os.Stderr, "DEBUG(config.Step): after %s\n", data)
}

func (a *A) SetP(p vector.V) { a.O.P = p }

func (a *A) MarshalJSON() ([]byte, error) {
	return json.Marshal(&O{
		P:            a.P(),
		V:            a.V(),
		R:            a.R(),
		Goal:         a.Goal(),
		Heading:      a.Heading(),
		Mass:         a.Mass(),
		MaxVelocity:  a.MaxVelocity(),
		MaxNetForce:  a.MaxNetForce(),
		MaxNetTorque: a.MaxNetTorque(),
	})
}

func (a *A) UnmarshalJSON(data []byte) error { return json.Unmarshal(data, &a.O) }

type C struct {
	Agents []*A
	Height float64
	Width  float64
}
