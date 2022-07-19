package locomotion

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

type L interface {
	Mutate(a agent.RW, acceleration vector.V, tau float64)
}

var _ L = impl{}

type impl struct{}

func New() L { return impl{} }

func (l impl) Mutate(a agent.RW, acceleration vector.V, tau float64) {
	a.SetV(agent.Clamp(vector.Add(a.V(), vector.Scale(tau, acceleration)), 0, a.MaxSpeed()))
	a.SetP(vector.Add(a.P(), vector.Scale(tau, a.V())))

	if !vector.Within(a.V(), *vector.New(0, 0)) {
		// Set the heading parallel to the last moved tick direction.
		a.SetHeading(*polar.New(1, polar.Polar(vector.Scale(tau, a.V())).Theta()))
	}
}
