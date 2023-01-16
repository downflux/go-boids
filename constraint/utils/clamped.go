package utils

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func Clamped(steerings []constraint.Steer, limit float64) constraint.Steer {
	return func(a agent.RO) vector.V {
		accumulator := 0.0
		result := vector.M{0, 0}

		buf := vector.M{0, 0}

		for _, s := range steerings {
			buf.Copy(s(a))
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
