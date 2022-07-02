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
	// PoolSize is the number of workers that will process the the agents in
	// parallel. We want this to be on the order of magnitude of the number
	// of cores on the system for fastest processing times.
	PoolSize int

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

type result struct {
	Mutation Mutation
	Error    error
}

// Step iterates through a single simulation step, but does not mutate the given
// state.
func Step(o O) []Mutation {
	agents := kd.Agents(kd.Data(o.T))

	// TODO(minkezhang): Find a better way to get the global variable here.
	r := 0.0
	for _, a := range agents {
		r = math.Max(r, a.R())
	}

	ach := make(chan agent.RO, 8*o.PoolSize)
	mch := make(chan Mutation, 8*o.PoolSize)

	go func(ch chan<- agent.RO) {
		defer close(ch)
		for _, a := range agents {
			ch <- a
		}
	}(ach)

	n := int(math.Min(float64(len(agents)), float64(o.PoolSize)))

	// Start up a number of workers to find the iterative velocity in
	// parallel.
	opts := base.O{
		T:   o.T,
		R:   r,
		Tau: o.Tau,

		CollisionWeight: o.CollisionWeight,
		CollisionFilter: o.CollisionFilter,

		ArrivalWeight: o.ArrivalWeight,
	}
	for i := 0; i < n; i++ {
		go func(ich <-chan agent.RO, och chan<- Mutation) {
			for a := range ich {
				och <- Mutation{
					Agent:    a,
					Steering: steering.S(a, base.New(opts).Force(a), o.Tau),
				}
			}
		}(ach, mch)
	}

	mutations := make([]Mutation, 0, len(agents))
	for i := 0; i < len(agents); i++ {
		mutations = append(mutations, <-mch)
	}
	return mutations
}
