package arrival

import (
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C struct {
	constraint.Base
	o O
}

type O struct {
}

func New(o O) *C {
	return &C{
		Base: *constraint.New("arrival"),
		o:    o,
	}
}

func (c C) Accelerate(a agent.RO) vector.V {
	r := vector.Sub(a.Goal(), a.P())
	d := vector.Magnitude(r)

	if vector.Within(r, *vector.New(0, 0)) {
		return *vector.New(0, 0)
	}

	// Once the agent arrives at the goal, we want to ensure no oscillating
	// behavior occurs -- this is done via a dampening factor on the
	// seek-like behavior.
	//
	// See
	// http://documentation.particleflocker.com/?docs=flocking-systems/steering-behaviours/arrival
	// or Park, Tahk, and Bang (2004) for more information.
	dampening := 1.0
	if a.R() > 0 && d < a.R() {
		dampening = math.Max(0, d/a.R()-0.5)
	}

	if vector.Within(r, *vector.New(0, 0)) {
		return *vector.New(0, 0)
	}

	return vector.Scale(dampening, vector.Unit(r))
}
