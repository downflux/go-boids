package config

import (
	"encoding/json"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.A = &A{}

type O struct {
	P           vector.V
	V           vector.V
	R           float64
	Goal        vector.V
	MaxNetForce float64
	MaxSpeed    float64
}

type A struct {
	O
}

func (a *A) P() vector.V          { return a.O.P }
func (a *A) V() vector.V          { return a.O.V }
func (a *A) R() float64           { return a.O.R }
func (a *A) Goal() vector.V       { return a.O.Goal }
func (a *A) MaxSpeed() float64    { return a.O.MaxSpeed }
func (a *A) MaxNetForce() float64 { return a.O.MaxNetForce }

func (a *A) SetP(p vector.V) { a.O.P = p }
func (a *A) SetV(v vector.V) { a.O.V = v }

func (a *A) MarshalJSON() ([]byte, error) {
	return json.Marshal(&O{
		P:           a.P(),
		V:           a.V(),
		R:           a.R(),
		Goal:        a.Goal(),
		MaxSpeed:    a.MaxSpeed(),
		MaxNetForce: a.MaxNetForce(),
	})
}

func (a *A) UnmarshalJSON(data []byte) error { return json.Unmarshal(data, &a.O) }

type C struct {
	Agents []*A
}
