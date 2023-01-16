package mock

import (
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

func M(v vector.V) constraint.Steer {
	return func(a agent.RO) vector.V { return v }
}
