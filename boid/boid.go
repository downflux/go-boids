package boid

import (
	"fmt"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/constraint/base"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

	cc "github.com/downflux/go-boids/contrib/constraint/collision"
	cst "github.com/downflux/go-boids/contrib/constraint/steering/target"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	T *kd.T

	Tau float64

	F func(a agent.A) bool
}

type Mutation struct {
	Agent        agent.A
	Acceleration v2d.V
}

func Step(o O) []Mutation {
	var mutations []Mutation
	for _, a := range kd.Agents(kd.Data(o.T)) {
		neighbors, err := kd.RadialFilter(
			o.T,
			*hypersphere.New(
				vector.V(a.P()),
				o.Tau*a.MaxSpeed()+3*a.R(),
			),
			// TODO(minkezhang): Check for interface equality
			// instead of coordinate equality, via adding an
			// Agent.Equal function.
			//
			// This technically may introduce a bug when multiple
			// points are extremely close together.
			func(p kd.P) bool {
				return !vector.Within(p.P(), vector.V(a.P())) && o.F(p.(kd.P).Agent())
			},
		)
		if err != nil {
			panic(fmt.Sprintf("could not generate simulation step: %v", err))
		}

		var cs []constraint.C

		var obstacles []agent.A
		for _, b := range kd.Agents(neighbors) {
			obstacles = append(obstacles, b)
		}

		cs = append(cs,
			cc.New(cc.O{
				Obstacles: obstacles,
				K:         10,
				Tau:       o.Tau,
			}),
			cst.New(cst.O{
				K:   2,
				Tau: o.Tau,
			}),
		)
		mutations = append(mutations, Mutation{
			Agent:        a,
			Acceleration: v2d.Scale(o.Tau, base.New(cs).A(a)),
		})
	}

	return mutations
}
