package mock

import (
	"encoding/json"
	"fmt"
	"log"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	_ agent.RW = &A{}

	// MaxNetForce is the maximum impuse that can be generated over some
	// time period tau. Note that this should be fairly large compared to
	// MaxSpeed to ensure agents can stop in time to avoid collisions.
	//
	// We are using a Lambo as a rough guess of an agent scale. Such a car
	// has a mass of ~2000 kg, and has a maximum acceleration of ~10 m/s/s.
	// Thus, we would expect the maximum thrust generated by this car to be
	// ~20 kN. At 100 km/hr, the car would be traveling at ~30 m/s -- we can
	// be generous here and increase the maximum speed.
	MaxNetForce = 20000.0
	Mass        = 2000.0
	MaxSpeed    = 30.0
	Radius      = 4.0
)

type DebugID string

type A struct {
	o   *O
	Log *log.Logger
}

type O struct {
	ID      DebugID
	P       vector.V
	V       vector.V
	Goal    vector.V
	Heading polar.V
	R       float64

	Mass        float64
	MaxNetForce float64
	MaxSpeed    float64
}

func Lamborghini(o O) *A {
	heading := o.Heading
	if heading == nil {
		heading = *polar.New(1, 0)
		if !vector.Within(o.V, *vector.New(0, 0)) {
			heading = polar.Polar(vector.Unit(o.V))
		}
	}
	return New(O{
		ID:          o.ID,
		P:           o.P,
		V:           o.V,
		R:           Radius,
		Goal:        o.Goal,
		Mass:        Mass,
		MaxNetForce: MaxNetForce,
		MaxSpeed:    MaxSpeed,
		Heading:     heading,
	})
}

func New(o O) *A {
	a := &A{
		o: &o,
	}
	if err := agent.Validate(a); err != nil {
		panic(fmt.Sprintf("cannot construct mock agent: %v", err))
	}
	return a
}

func (a *A) DebugID() DebugID { return a.o.ID }

func (a *A) P() vector.V         { return a.o.P }
func (a *A) V() vector.V         { return a.o.V }
func (a *A) R() float64          { return a.o.R }
func (a *A) Heading() polar.V    { return a.o.Heading }
func (a *A) Goal() vector.V      { return a.o.Goal }
func (a *A) Logger() *log.Logger { return a.Log }
func (a *A) MaxNetAcceleration() float64 {
	if a.o.Mass == 0 {
		panic("cannot find max acceleration for a mock object with zero mass")
	}
	return a.o.MaxNetForce / a.o.Mass
}
func (a *A) MaxSpeed() float64 {
	cutoff := 1.5 * a.R()
	if d := vector.Magnitude(vector.Sub(a.P(), a.Goal())); d < cutoff {
		return (d / cutoff) * a.o.MaxSpeed
	}
	return a.o.MaxSpeed
}

func (a *A) SetP(v vector.V)      { a.o.P = v }
func (a *A) SetV(v vector.V)      { a.o.V = v }
func (a *A) SetHeading(v polar.V) { a.o.Heading = v }

func (a *A) MarshalJSON() ([]byte, error) {
	return json.Marshal(&O{
		ID:          a.DebugID(),
		P:           a.P(),
		V:           a.V(),
		R:           a.R(),
		Mass:        a.o.Mass,
		Goal:        a.Goal(),
		MaxSpeed:    a.MaxSpeed(),
		MaxNetForce: a.o.MaxNetForce,
		Heading:     a.Heading(),
	})
}

func (a *A) UnmarshalJSON(data []byte) error {
	if err := json.Unmarshal(data, &a.o); err != nil {
		return err
	}
	return agent.Validate(a)
}
