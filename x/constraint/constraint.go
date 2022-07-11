package constraint

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

type C interface {
	// Accelerate is a function which returns the desired net acceleration
	// for a specific set of forces. For seek behavior, the acceleration
	// vector will point directly to the target.
	Accelerate(a agent.RO) vector.V
}

type steer struct {
	f func(a agent.RO) vector.V
	w float64
}

func (s steer) Accelerate(a agent.RO) vector.V {
	return vector.Scale(s.w, agent.Steer(a, s.f(a), 1))
}

func Steer(c C, weight float64) C {
	return steer{
		f: c.Accelerate,
		w: weight,
	}
}
