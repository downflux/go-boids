package clamped

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Clamped(accelerators []constraint.Accelerator, limit float64) constraint.Accelerator {
	accumulator := 0.0
	result := vector.M{0, 0}

	buf := vector.M{0, 0}

	return func(a agent.RO) vector.V {
		for _, accel := range accelerators {
			buf.Copy(accel(a))
			m := vector.Magnitude(buf.V())
			if m > limit-accumulator {
				m = limit - accumulator
				buf.Unit()
				buf.Scale(m)
				result.Add(buf.V())
				break
			}
			accumulator += m
			result.Add(buf.V())
		}
		return result.V()
	}
}
