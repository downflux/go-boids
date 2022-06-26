package base

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/internal/accumulator"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type C []constraint.C

func New(constraints []constraint.C) *C {
	c := C(constraints)
	return &c
}

func (c C) Force(a agent.A) vector.V {
	v := *vector.New(0, 0)
	acc := accumulator.New(a.MaxNetForce())

	for _, constraint := range c {
		acceleration, ok := acc.Add(constraint.Force(a))
		v = vector.Add(v, acceleration)
		if !ok {
			break
		}
	}

	if vector.Within(v, *vector.New(0, 0)) {
		return *vector.New(0, 0)
	}
	return vector.Scale(a.MaxNetForce(), vector.Unit(v))
}
