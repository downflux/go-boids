package agent

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.RO = &Mock{}

type O struct {
	ID   agent.ID
	P    vector.V
	V    vector.V
	R    float64
	Mass float64
	Goal vector.V

	Heading polar.V

	MaxVelocity polar.V

	MaxNetForce  float64
	MaxNetTorque float64
}

type Mock struct {
	O
}

func New(o O) *Mock {
	return &Mock{O: o}
}

func (a *Mock) ID() agent.ID          { return a.O.ID }
func (a *Mock) P() vector.V           { return a.O.P }
func (a *Mock) V() vector.V           { return a.O.V }
func (a *Mock) R() float64            { return a.O.R }
func (a *Mock) Goal() vector.V        { return a.O.Goal }
func (a *Mock) Heading() polar.V      { return a.O.Heading }
func (a *Mock) MaxVelocity() polar.V  { return a.O.MaxVelocity }
func (a *Mock) MaxNetForce() float64  { return a.O.MaxNetForce }
func (a *Mock) MaxNetTorque() float64 { return a.O.MaxNetTorque }
func (a *Mock) Mass() float64         { return a.O.Mass }
