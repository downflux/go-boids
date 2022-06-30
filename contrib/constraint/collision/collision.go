package collision

import (
	"fmt"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/constraint/weighted"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

	ca "github.com/downflux/go-boids/contrib/constraint/collision/agent"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C struct {
	o O
}

type O struct {
	T *kd.T
	K float64

	Cutoff float64
	Filter func(a agent.RO) bool
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

// TOOD(minkezhang): Toy with using a PQ here instead of a weighted average.
func (c C) Force(a agent.RO) v2d.V {
	var cs []constraint.C
	var ws []float64

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

	for _, o := range neighbors {
		cs = append(cs, ca.New(ca.O{
			K:        c.o.K,
			Obstacle: o.Agent(),
		}))
		ws = append(ws, 1.0/v2d.Magnitude(v2d.Sub(a.P(), o.Agent().P())))
	}
	return weighted.New(cs, ws).Force(a)
}
