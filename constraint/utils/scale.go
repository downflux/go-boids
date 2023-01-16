package utils

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Scale(f float64, s constraint.Steer) constraint.Steer {
	return func(a agent.RO) vector.V {
		return vector.Scale(f, s(a))
	}
}
