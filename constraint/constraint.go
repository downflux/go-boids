package constraint

import (
	"fmt"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ C = steer{}

type Base struct {
	name string
}

func (b Base) Name() string { return b.name }

func New(name string) *Base {
	return &Base{
		name: name,
	}
}

type C interface {
	Name() string

	// Accelerate is a function which returns the desired net acceleration
	// for a specific set of forces. For seek behavior, the acceleration
	// vector will point directly to the target.
	Accelerate(a agent.RO) vector.V
}

type steer struct {
	Base
	f func(a agent.RO) vector.V
	w float64
	c float64
}

func (s steer) Accelerate(a agent.RO) vector.V {
	return agent.Clamp(
		vector.Scale(s.w, agent.Steer(a, s.f(a), 1)),
		0,
		s.c,
	)
}

func Steer(c C, weight float64, clamp float64) C {
	return steer{
		Base: *New(fmt.Sprintf("steer_%v", c.Name())),

		f: c.Accelerate,
		w: weight,
		c: clamp,
	}
}
