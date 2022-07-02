package boid

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/constraint/truncated"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-boids/contrib/steering"

	ca "github.com/downflux/go-boids/contrib/constraint/arrival"
	cc "github.com/downflux/go-boids/contrib/constraint/collision"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	T   *kd.T
	Tau float64

	F func(a agent.RO) bool
}

type Mutation struct {
	Agent agent.RO

	// Steering is the desired velocity for the next tick passed back to the
	// agent. The agent is responsible for fulfilling this velocity request
	// via the locomotion layer.
	Steering v2d.V
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
				K:      15,
				Cutoff: o.Tau*a.MaxVelocity().R() + 5*r,
				Filter: o.F,
			}),
			ca.New(ca.O{
				K: 6,
			}),
		}

		mutations = append(mutations, Mutation{
			Agent:    a,
			Steering: steering.S(a, truncated.New(cs).Force(a), o.Tau),
		})
	}

	return mutations
}
