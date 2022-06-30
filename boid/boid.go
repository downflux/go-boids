package boid

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/constraint/truncated"
	"github.com/downflux/go-boids/kd"

	ca "github.com/downflux/go-boids/contrib/constraint/arrival"
	cc "github.com/downflux/go-boids/contrib/constraint/collision"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	T   *kd.T
	Tau float64

	F func(a agent.A) bool
}

type Mutation struct {
	Agent    agent.A
	Steering v2d.V

	// Visualzation-only fields.
	Acceleration v2d.V
}

// Step iterates through a single simulation step, but does not mutate the given
// state.
//
// TODO(minkezhang): Make this concurrent.
func Step(o O) []Mutation {
	var mutations []Mutation
	// TODO(minkezhang): Find a better way to get the global variable here.
	r := 0.0
	for _, a := range kd.Agents(kd.Data(o.T)) {
		r = math.Max(r, a.R())
	}

	for _, a := range kd.Agents(kd.Data(o.T)) {
		cs := []constraint.C{
			cc.New(cc.O{
				T:      o.T,
				K:      50,
				Cutoff: o.Tau*a.MaxVelocity().R() + 5*r,
				Filter: o.F,
			}),
			ca.New(ca.O{
				K: 6,
			}),
		}

		mutations = append(mutations, Steer(a, truncated.New(cs).Force(a), o.Tau))
	}

	return mutations
}

// Steer returns the desired velocity for the next tick for a given agent with
// the calculated input Boid force.
func Steer(a agent.A, force v2d.V, tau float64) Mutation {
	desired := *v2d.New(0, 0)
	if !v2d.Within(force, *v2d.New(0, 0)) {
		// Agent's locomotion directive is to travel as fast as possible
		// in the direction indicated by the force vector.
		desired = v2d.Scale(a.MaxVelocity().R(), v2d.Unit(force))
	}

	steering := *v2d.New(0, 0)
	if !v2d.Within(desired, a.V()) {
		steering = v2d.Scale(
			math.Min(
				tau*a.MaxNetForce()/a.Mass(),
				v2d.Magnitude(v2d.Sub(desired, a.V())),
			),
			v2d.Unit(v2d.Sub(desired, a.V())),
		)
	}

	return Mutation{
		Agent:        a,
		Steering:     steering,
		Acceleration: v2d.Scale(1/tau, desired),
	}
}
