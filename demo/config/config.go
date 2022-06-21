package config

import (
	"encoding/json"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/cylindrical"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.A = &A{}

type O struct {
	P vector.V
	V vector.V
	A cylindrical.V
	R float64
}

type A struct {
	O
}

func (a *A) P() vector.V                    { return a.O.P }
func (a *A) V() vector.V                    { return a.O.V }
func (a *A) R() float64                     { return a.O.R }
func (a *A) MaxAcceleration() cylindrical.V { return a.O.A }

func (a *A) MarshalJSON() ([]byte, error) {
	return json.Marshal(&O{
		P: a.P(),
		V: a.V(),
		A: a.MaxAcceleration(),
		R: a.R(),
	})
}

func (a *A) UnmarshalJSON(data []byte) error { return json.Unmarshal(data, &a.O) }

type C struct {
	Agents []agent.A
}
