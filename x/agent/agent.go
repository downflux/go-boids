package agent

import (
	"fmt"
	"time"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

func Tau(t time.Duration) float64 { return float64(t) / float64(time.Second) }

func Step(a RW, acceleration vector.V, tau float64) {
	a.SetV(vector.Add(a.V(), vector.Scale(tau, acceleration)))
	a.SetP(vector.Add(a.P(), vector.Scale(tau, a.V())))

	if !vector.Within(a.V(), *vector.New(0, 0)) {
		// Set the heading parallel to the last moved tick direction.
		a.SetHeading(*polar.New(1, polar.Polar(vector.Scale(tau, a.V())).Theta()))
	}
}

func Validate(a RO) error {
	if a.P() == nil {
		return fmt.Errorf("agent position must be non-nil")
	}
	if a.V() == nil {
		return fmt.Errorf("agent velocity must be non-nil")
	}
	if a.R() <= 0 {
		return fmt.Errorf("agent radius must be a positive value, but got %v", a.R())
	}
	if a.Heading() == nil {
		return fmt.Errorf("agent heading must be non-nil")
	}
	return nil
}

type RO interface {
	P() vector.V
	V() vector.V
	R() float64

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
