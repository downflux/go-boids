package agent

import (
	"fmt"
	"log"
	"time"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

func Tau(t time.Duration) float64 { return float64(t) / float64(time.Second) }

// Clamp ensures a 2D vector lies within the given bounds.
//
// TODO(minkezhang): Implement angular bounds check.
func Clamp(v vector.V, min float64, max float64) vector.V {
	m := vector.Magnitude(v)
	if epsilon.Within(m, 0) {
		return *vector.New(0, 0)
	}
	if m < min {
		m = min
	} else if m > max {
		m = max
	}
	return vector.Scale(m, vector.Unit(v))
}

// Steer returns a steering acceleration for the agent, given a desired heading
// vector.
//
// Note that the returned acceleration is not clamped -- the caller is
// responsible for additional weighting and clamping on the returned values.
//
// If no heading is provided, we sensibly return no acceleration -- that is, the
// agent should continue on its current trajectory.
func Steer(a RO, heading vector.V, tau float64) vector.V {
	if epsilon.Within(vector.SquaredMagnitude(heading), 0) {
		return *vector.New(0, 0)
	}
	desired := vector.Scale(tau*a.MaxSpeed(), vector.Unit(heading))
	return vector.Sub(desired, a.V())
}

func Step(a RW, acceleration vector.V, tau float64) {
	a.SetV(Clamp(vector.Add(a.V(), vector.Scale(tau, acceleration)), 0, a.MaxSpeed()))
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
	if a.Goal() == nil {
		return fmt.Errorf("agent goal must be non-nil")
	}
	if a.R() <= 0 {
		return fmt.Errorf("agent radius must be a positive value, but got %v", a.R())
	}
	if a.Heading() == nil {
		return fmt.Errorf("agent heading must be non-nil")
	}
	if a.MaxSpeed() < 0 {
		return fmt.Errorf("agent must have a non-negative max speed, but got %v", a.MaxSpeed())
	}
	if a.MaxNetAcceleration() < 0 {
		return fmt.Errorf("agent must have a non-negative max net acceleration, but got %v", a.MaxNetAcceleration())
	}
	return nil
}

type RO interface {
	P() vector.V
	V() vector.V
	R() float64
	Goal() vector.V
	Heading() polar.V
	Logger() *log.Logger

	MaxSpeed() float64
	MaxNetAcceleration() float64
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
