package mock

import (
	"encoding/json"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.RW = &A{}

type DebugID string

type A struct {
	o *O
}

type O struct {
	ID      DebugID
	P       vector.V
	V       vector.V
	R       float64
	Heading polar.V
}

func Lamborghini(o O) *A {
	return New(O{
		ID:      o.ID,
		P:       o.P,
		V:       o.V,
		R:       o.R,
		Heading: o.Heading,
	})
}

func New(o O) *A {
	return &A{
		o: &o,
	}
}

func (a *A) DebugID() DebugID { return a.o.ID }

func (a *A) P() vector.V      { return a.o.P }
func (a *A) V() vector.V      { return a.o.V }
func (a *A) R() float64       { return a.o.R }
func (a *A) Heading() polar.V { return a.o.Heading }

func (a *A) SetP(v vector.V)      { a.o.P = v }
func (a *A) SetV(v vector.V)      { a.o.V = v }
func (a *A) SetHeading(v polar.V) { a.o.Heading = v }

func (a *A) MarshalJSON() ([]byte, error) {
	return json.Marshal(&O{
		ID:      a.DebugID(),
		P:       a.P(),
		V:       a.V(),
		R:       a.R(),
		Heading: a.Heading(),
	})
}

func (a *A) UnmarshalJSON(data []byte) error { return json.Unmarshal(data, &a.o) }
