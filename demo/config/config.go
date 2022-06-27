package config

import (
	"encoding/json"
	"math"

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
// The angular component of the input vector is relative to the agent heading.
func (a *A) Step(v polar.V, tau float64) {
	a.O.Heading = *polar.New(1, a.O.Heading.Theta()+tau*math.Copysign(
		math.Min(a.MaxVelocity().Theta(), math.Abs(v.Theta())), v.Theta(),
	))

	a.O.P = vector.Add(a.P(), vector.Scale(tau, a.V()))
}

func (a *A) SetP(p vector.V)      { a.O.P = p }
func (a *A) SetV(v vector.V)      { a.O.V = v }
func (a *A) SetHeading(h polar.V) { a.O.Heading = h }

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
}
