package boid

import (
	"fmt"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/accumulator"
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
	Agent    agent.A
	Velocity v2d.V
}

func Step(o O) []Mutation {
	var mutations []Mutation
	for _, a := range kd.Agents(kd.Data(o.T)) {
		neighbors, err := kd.RadialFilter(
			o.T,
			*hypersphere.New(
				vector.V(a.P()),
				o.Tau*a.MaxVelocity().R()+3*a.R(),
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
				// K is the agent-agent collision scalar; a
				// larger value here allows the repulsive
				// acceleration to be larger, earlier, which
				// smooths out collisions as the acceleration
				// becomes noticeable from further away.
				K:   20,
				Tau: o.Tau,
			}),
			cst.New(cst.O{
				K:   1,
				Tau: o.Tau,
			}),
		)

		v := v2d.Add(a.V(), v2d.Scale(o.Tau, base.New(cs).A(a)))
		// u is the output velocity scaled by the agent velocity limits.
		// If the turning angle induced by the acceleration is too
		// large, the actual v is scaled back.
		//
		// TODO(minkezhang): Ensure the turning radius cannot reverse
		// (e.g. if max angular velocity is 0, there is only one degree
		// of freedom).
		//
		// See
		// https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-seek--gamedev-849
		// and Kakaria (2016) for more details.
		u, _ := accumulator.New(
			a.MaxVelocity(),
			a.V(),
		).Add(v)

		mutations = append(mutations, Mutation{
			Agent:    a,
			Velocity: u,
		})
	}

	return mutations
}
