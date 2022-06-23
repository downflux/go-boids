package target

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"

	e "github.com/downflux/go-geometry/epsilon"
)

const (
	epsilon  = 1e-3
	strength = 2
)

var _ constraint.C = C{}

type C struct{}
type O struct{}

func New(o O) *C {
	return &C{}
}

func (c C) Priority() constraint.P { return 0 }

func (C C) A(a agent.A) vector.V {
	r := vector.Sub(a.Goal(), a.P())
	d := vector.Magnitude(r)
	if e.Within(d, 0) {
		return *vector.New(0, 0)
	}
	// TODO(minkezhang): Implement "Arrival" behavior, e.g.
	// http://documentation.particleflocker.com/?docs=flocking-systems/steering-behaviours/arrival.
	//
	// See also Park, Tahk, and Bang 2004.
	if d < a.R() {
	}
	return vector.Scale(strength, vector.Unit(r))
}
