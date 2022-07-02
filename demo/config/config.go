package config

import (
	"encoding/json"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/contrib/locomotion"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.RW = &A{}

type O struct {
	ID           agent.ID
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

func (a *A) ID() agent.ID         { return a.O.ID }
func (a *A) P() vector.V          { return a.O.P }
func (a *A) V() vector.V          { return a.O.V }
func (a *A) R() float64           { return a.O.R }
func (a *A) Goal() vector.V       { return a.O.Goal }
func (a *A) Heading() polar.V     { return a.O.Heading }
func (a *A) Mass() float64        { return a.O.Mass }
func (a *A) MaxVelocity() polar.V { return a.O.MaxVelocity }
func (a *A) MaxAcceleration() polar.V {
	return *polar.New(
		a.O.MaxNetForce/a.Mass(),
		a.O.MaxNetTorque/(0.5*a.Mass()*a.R()*a.R()),
	)
}

// Step advances the Boid simulation by a single step.
func (a *A) Step(steering vector.V, tau float64) {
	b := locomotion.L(a, steering, tau)

	a.O.P = b.P()
	a.O.Heading = b.Heading()
	a.O.V = b.V()
}

func (a *A) SetP(p vector.V) { a.O.P = p }

func (a *A) MarshalJSON() ([]byte, error) {
	return json.Marshal(&O{
		ID:           a.ID(),
		P:            a.P(),
		V:            a.V(),
		R:            a.R(),
		Goal:         a.Goal(),
		Heading:      a.Heading(),
		Mass:         a.Mass(),
		MaxVelocity:  a.MaxVelocity(),
		MaxNetForce:  a.O.MaxNetForce,
		MaxNetTorque: a.O.MaxNetTorque,
	})
}

func (a *A) UnmarshalJSON(data []byte) error { return json.Unmarshal(data, &a.O) }

type C struct {
	Agents []*A
	Height float64
	Width  float64
}
