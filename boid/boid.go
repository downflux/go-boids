package boid

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/contrib/constraint/base"
	"github.com/downflux/go-boids/contrib/steering"
	"github.com/downflux/go-boids/kd"

	v2d "github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	T   *kd.T
	Tau float64

	CollisionWeight float64
	CollisionFilter func(a agent.RO) bool

	ArrivalWeight float64
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
		mutations = append(mutations, Mutation{
			Agent: a,
			Steering: steering.S(
				a,
				base.New(base.O{
					T:   o.T,
					R:   r,
					Tau: o.Tau,

					CollisionWeight: o.CollisionWeight,
					CollisionFilter: o.CollisionFilter,

					ArrivalWeight: o.ArrivalWeight,
				}).Force(a),
				o.Tau,
			),
		})
	}

	return mutations
}
