package cohesion

import (
	"fmt"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/weighted"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

	ca "github.com/downflux/go-boids/constraint/contrib/cohesion/agent"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C struct {
	o O
}

type O struct {
	T *kd.T

	Cutoff float64
	Filter func(a agent.RO) bool
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

// Accelerate returns a net desired acceleration pointing towards the best-guess
// collision-free zone. The collision constraint here will prioritize the
// closest collisions and ignore any additional acceleration requests if the
// allocated acceleration bin has been saturated.
func (c C) Accelerate(a agent.RO) v2d.V {
	neighbors, err := kd.RadialFilter(
		c.o.T,
		*hypersphere.New(vector.V(a.P()), c.o.Cutoff),
		// TODO(minkezhang): Check for interface equality instead of
		// coordinate equality, via adding an Agent.Equal function.
		//
		// This technically may introduce a bug when multiple points are
		// extremely close together.
		func(p kd.P) bool {
			return !vector.Within(p.P(), vector.V(a.P())) && c.o.Filter(p.(kd.P).Agent())
		},
	)
	if err != nil {
		panic(fmt.Sprintf("could not find neighbors for KD-tree: %v", err))
	}

	var cs []constraint.C
	var ws []float64
	for _, o := range neighbors {
		cs = append(cs, ca.New(ca.O{
			Obstacle: o.Agent(),
		}))
		ws = append(ws, 1)
	}

	return weighted.New(cs, ws).Accelerate(a)
}
