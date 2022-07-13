package alignment

import (
	"fmt"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

	v2d "github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type O struct {
	T *kd.T

	Cutoff float64
	Filter func(a agent.RO) bool
}

type C struct {
	o O
}

func New(o O) *C {
	return &C{
		o: o,
	}
}

func (c C) Accelerate(a agent.RO) v2d.V {
	neighbors, err := kd.RadialFilter(
		c.o.T,
		*hypersphere.New(vector.V(a.P()), c.o.Cutoff),
		func(p kd.P) bool {
			return !vector.Within(p.P(), vector.V(a.P())) && c.o.Filter(p.(kd.P).Agent())
		},
	)
	if err != nil {
		panic(fmt.Sprintf("could not find neighbors for KD-tree: %v", err))
	}

	result := *v2d.New(0, 0)
	for _, n := range neighbors {
		result = v2d.Add(result, v2d.Scale(1.0/float64(len(neighbors)), n.Agent().V()))
	}

	return result
}
