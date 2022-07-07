package mock

import (
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.RW = &A{}

type A struct {
	o *O
}

type O struct {
	P       vector.V
	V       vector.V
	Heading polar.V
}

func New(o O) *A {
	return &A{
		o: &o,
	}
}

func (a *A) P() vector.V      { return a.o.P }
func (a *A) V() vector.V      { return a.o.V }
func (a *A) Heading() polar.V { return a.o.Heading }

func (a *A) SetP(v vector.V)      { a.o.P = v }
func (a *A) SetV(v vector.V)      { a.o.V = v }
func (a *A) SetHeading(v polar.V) { a.o.Heading = v }
