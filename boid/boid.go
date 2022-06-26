package boid

import (
	"fmt"
	"math"

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
	Agent    agent.A
	Velocity v2d.V
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
		neighbors, err := kd.RadialFilter(
			o.T,
			*hypersphere.New(
				vector.V(a.P()),
				o.Tau*a.MaxSpeed()+3*r,
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
				K:         2,
				Tau:       o.Tau,
			}),
			cst.New(cst.O{
				K:   1,
				Tau: o.Tau,
			}),
		)

		mutations = append(mutations, Mutation{
			Agent:    a,
			Velocity: v2d.Scale(o.Tau, Steer(a, base.New(cs).Force(a))),
		})
	}

	return mutations
}

// Steer returns the desired velocity for the next tick for a given agent with
// the calculated input Boid force.
func Steer(a agent.A, force v2d.V) v2d.V {
	// Tau is a pseudo constant with units of time. This allows unit
	// matching, which makes the overall code easier to read.
	const tau = 1.0

	// TODO(minkezhang): Implement agent heading API.
	steering := v2d.Scale(tau, v2d.Sub(v2d.Scale(tau/a.Mass(), force), a.V()))
	if v2d.Within(steering, *v2d.New(0, 0)) {
		return *v2d.New(0, 0)
	}

	steering = v2d.Scale(
		math.Min(
			a.MaxNetForce(),
			v2d.Magnitude(steering),
		),
		v2d.Unit(steering),
	)

	v := v2d.Add(a.V(), v2d.Scale(tau, steering))
	if v2d.Within(v, *v2d.New(0, 0)) {
		return *v2d.New(0, 0)
	}

	return v2d.Scale(
		math.Min(
			a.MaxSpeed(),
			v2d.Magnitude(v),
		),
		v2d.Unit(v),
	)
}
