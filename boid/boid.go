package boid

import (
	"fmt"
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/constraint/truncated"
	"github.com/downflux/go-boids/kd"
	// "github.com/downflux/go-geometry/2d/line"
	// "github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

	ca "github.com/downflux/go-boids/contrib/constraint/arrival"
	cc "github.com/downflux/go-boids/contrib/constraint/collision"
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
	Heading  polar.V

	// Visualzation-only fields.
	Acceleration v2d.V
	Steering     v2d.V
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
				o.Tau*a.MaxVelocity().R()+3*r,
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
			ca.New(ca.O{
				K:   1,
				Tau: o.Tau,
			}),
		)

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
	steering := v2d.Scale(second, v2d.Sub(desired, a.V()))

	if v2d.Within(steering, *v2d.New(0, 0)) {
		return Mutation{
			Agent:    a,
			Velocity: a.V(),
			Heading:  a.Heading(),

			Steering:     steering,
			Acceleration: acceleration,
		}
	}

	steering = v2d.Scale(
		math.Min(
			a.MaxNetForce(),
			v2d.Magnitude(steering),
		),
		v2d.Unit(steering),
	)

	/*
		const omega = math.Pi / 8
		s := *segment.New(*line.New(a.V(), steering), 0, 1)

		for _, theta := range []float64{omega, -omega} {
			// u is the current heading rotated the maximum allowable
			// angular velocity. We use this to check for intersections with
			// the steering force. If there is an intersection with the
			// force vector, this means the resultant velocity exceeds the
			// maximum turn -- we will need to scale back the steering force
			// accordingly.
			u := *line.New(
				*v2d.New(0, 0),
				v2d.Rotate(theta, a.Heading()),
			)
			if p, ok := s.L().Intersect(u); ok && s.L().T(p) > s.TMin() && s.L().T(p) < s.TMax() {
				steering = v2d.Sub(p, a.V())
				if v2d.Within(a.V(), *v2d.New(0, 0)) {
					return Mutation{
						Agent:    a,
						Velocity: *v2d.New(0, 0),
						Heading:  v2d.Rotate(theta, a.Heading()),

						Steering: steering,
						Acceleration: acceleration,
					}
				}
			}
		}
	*/
	v := v2d.Add(a.V(), v2d.Scale(second, steering))
	if v2d.WithinEpsilon(v, *v2d.New(0, 0), epsilon.Relative(1e-5)) {
		return Mutation{
			Agent:    a,
			Velocity: *v2d.New(0, 0),
			Heading:  a.Heading(),

			Steering:     steering,
			Acceleration: acceleration,
		}
	}

	return Mutation{
		Agent: a,
		Velocity: v2d.Scale(
			tau*math.Min(
				a.MaxVelocity().R(),
				v2d.Magnitude(v),
			), v2d.Unit(v)),
		Heading: polar.Polar(v2d.Unit(v)),

		Steering:     steering,
		Acceleration: acceleration,
	}
}
