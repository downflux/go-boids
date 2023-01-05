package clamped

import (
	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

const (
	tolerance = 1e-5
)

func Clamped(c constraint.Accelerator) constraint.Accelerator {
	return func(a agent.RO) vector.V {
		u := vector.M{0, 0}
		u.Copy(c(a))
		if epsilon.Absolute(tolerance).Within(vector.SquaredMagnitude(u.V()), 0) {
			return vector.V{0, 0}
		}
		u.Unit()
		u.Scale(a.MaxVelocity())
		return u.V()
	}
}
