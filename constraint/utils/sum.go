package utils

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Sum(accelerators []constraint.Accelerator) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		result := vector.M{0, 0}
		for _, accel := range accelerators {
			result.Add(accel(a))
		}
		return result.V()
	}
}
