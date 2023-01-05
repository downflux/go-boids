package seek

import (
	"math"

	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

func Seek(a agent.RO) vector.V {
	r := vector.Sub(a.TargetPosition(), a.Position())
	d := vector.Magnitude(r)

	if epsilon.Absolute(1e-5).Within(d, 0) {
		return vector.V{0, 0}
	}

	// Once the agent arrives at the goal, we want to ensure no oscillating
	// behavior occurs -- this is done via a dampening factor on the
	// seek-like behavior.
	//
	// See
	// http://documentation.particleflocker.com/?docs=flocking-systems/steering-behaviours/arrival
	// or Park, Tahk, and Bang (2004) for more information.
	dampening := 1.0
	if d < a.Radius() {
		dampening = math.Max(0, d/a.Radius()-0.5)
	}

	return vector.Scale(dampening*a.MaxVelocity(), vector.Unit(r))
}
