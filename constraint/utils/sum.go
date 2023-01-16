package utils

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Sum(steerings []constraint.Steer) constraint.Steer {
	return func(a agent.RO) vector.V {
		result := vector.M{0, 0}
		for _, accel := range steerings {
			result.Add(accel(a))
		}
		return result.V()
	}
}
