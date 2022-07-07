package agent

import (
	"time"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

func Tau(t time.Duration) float64 { return float64(t) / float64(time.Second) }

func Step(a RW, acceleration vector.V, tau float64) {
	a.SetV(vector.Add(a.V(), vector.Scale(tau, acceleration)))
	a.SetP(vector.Add(a.P(), vector.Scale(tau, a.V())))

	if !vector.Within(a.V(), *vector.New(0, 0)) {
		a.SetHeading(*polar.New(1, polar.Polar(a.V()).Theta()))
	}
}

type RO interface {
	P() vector.V
	V() vector.V

	Heading() polar.V
}

type WO interface {
	SetP(v vector.V)
	SetV(v vector.V)

	SetHeading(v polar.V)
}

type RW interface {
	RO
	WO
}
