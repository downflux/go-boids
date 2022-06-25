package agent

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ agent.A = &Mock{}

type O struct {
	P vector.V
	V vector.V
	R float64

	Goal vector.V

	MaxSpeed        float64
	MaxAcceleration polar.V
}

type Mock struct {
	O
}

func New(o O) *Mock {
	return &Mock{O: o}
}

func (a *Mock) P() vector.V              { return a.O.P }
func (a *Mock) V() vector.V              { return a.O.V }
func (a *Mock) R() float64               { return a.O.R }
func (a *Mock) Goal() vector.V           { return a.O.Goal }
func (a *Mock) MaxSpeed() float64        { return a.O.MaxSpeed }
func (a *Mock) MaxAcceleration() polar.V { return a.O.MaxAcceleration }
