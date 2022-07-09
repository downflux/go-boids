package boid

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/kd"

	v2d "github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	// PoolSize is the number of workers that will process the the agents in
	// parallel. We want this to be on the order of magnitude of the number
	// of cores on the system for fastest processing times.
	PoolSize int

	T         *kd.T
	Tau       float64
	MaxRadius float64

	CollisionWeight float64
	CollisionFilter func(a agent.RO) bool

	ArrivalWeight float64
}

type Mutation struct {
	Agent agent.RO

	// Steering is the acceleration for the next tick passed back to the
	// agent. The agent is responsible for fulfilling this acceleration
	// request via the locomotion layer.
	Steering v2d.V
}

// Step iterates through a single simulation step, but does not mutate the given
// state.
func Step(o O) []Mutation {
	var ms []Mutation
	for _, p := range kd.Agents(kd.Data(o.T)) {
		ms = append(ms, Mutation{
			Agent:    p,
			Steering: agent.Steer(p, *v2d.New(1, 0), 1),
		})
	}
	return ms
}
