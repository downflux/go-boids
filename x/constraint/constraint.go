package constraint

import (
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

type Steer func(a agent.RO) vector.V
