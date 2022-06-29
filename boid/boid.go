package boid

import (
	"encoding/json"
	"fmt"
	"math"
	"os"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/accumulator"
	"github.com/downflux/go-boids/internal/constraint/truncated"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

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
		vision := o.Tau*a.MaxVelocity().R() + 7*r

		neighbors, err := kd.RadialFilter(
			o.T,
			*hypersphere.New(vector.V(a.P()), vision),
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
				K:         1,
				Tau:       o.Tau,
				MaxRange:  vision,
			}),
			ca.New(ca.O{
				K:   1,
				Tau: o.Tau,
			}),
		)

		data, _ := json.MarshalIndent(
			map[string]string{
				"collision": fmt.Sprintf("(%.3f, %.3f)", cs[0].Force(a).X(), cs[0].Force(a).Y()),
				"arrival":   fmt.Sprintf("(%.3f, %.3f)", cs[1].Force(a).X(), cs[1].Force(a).Y()),
			},
			"", "  ")
		fmt.Fprintf(os.Stderr, "DEBUG(boid.Step): %s\n", data)

		mutations = append(mutations, Steer(a, truncated.New(cs).Force(a), o.Tau))
	}

	return mutations
}

// Steer returns the desired velocity for the next tick for a given agent with
// the calculated input Boid force.
func Steer(a agent.A, force v2d.V, tau float64) Mutation {
	// second is a pseudo constant with units of time. This allows unit
	// matching, which makes the overall code easier to read.
	const second = 1.0

	acceleration := v2d.Scale(1/a.Mass(), force)
	desired := v2d.Scale(second, acceleration)
	steering, _ := accumulator.New(accumulator.D{
		Force:  a.MaxNetForce(),
		Torque: a.MaxNetTorque(),
	}, a.R(), a.Heading()).Add(
		v2d.Scale(second, v2d.Sub(desired, a.V())),
	)

	data, _ := json.MarshalIndent(
		map[string]string{
			"a.P()":    fmt.Sprintf("(%.3f, %.3f)", a.P().X(), a.P().Y()),
			"force":    fmt.Sprintf("(%.3f, %.3f)", force.X(), force.Y()),
			"steering": fmt.Sprintf("(%.3f, %.3f)", steering.X(), steering.Y()),
			"desired":  fmt.Sprintf("(%.3f, %.3f)", desired.X(), desired.Y()),
		},
		"", "  ")
	fmt.Fprintf(os.Stderr, "DEBUG(boid.Steer): %s\n", data)

	return Mutation{
		Agent:        a,
		Steering:     steering,
		Acceleration: acceleration,
	}
}
