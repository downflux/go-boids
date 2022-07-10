package collision

import (
	"fmt"
	"sort"

	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-boids/x/constraint/clamped"
	"github.com/downflux/go-boids/x/kd"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"

	ca "github.com/downflux/go-boids/x/constraint/contrib/collision/agent"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

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

type datum struct {
	c        constraint.Steer
	distance float64
}

type data []datum

func (d data) Len() int           { return len(d) }
func (d data) Swap(i, j int)      { d[i], d[j] = d[j], d[i] }
func (d data) Less(i, j int) bool { return d[i].distance < d[j].distance }

func (c C) Steer(a agent.RO) v2d.V {
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

	if len(neighbors) > 0 {
		a.Logger().Printf("DEBUG(collision.Steer): neighbors == %v", len(neighbors))
	}

	var ds []datum
	for _, o := range neighbors {
		ds = append(ds, datum{
			c: ca.New(ca.O{
				K:        c.o.K,
				Obstacle: o.Agent(),
			}).Steer,
			distance: v2d.SquaredMagnitude(v2d.Sub(a.P(), o.Agent().P())),
		})
	}
	sort.Sort(data(ds))

	var cs []constraint.Steer
	for _, d := range ds {
		cs = append(cs, d.c)
	}

	return clamped.New(cs).Steer(a)
}