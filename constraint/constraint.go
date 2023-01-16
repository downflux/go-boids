package constraint

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// Steer is a function which returns the desired net acceleration for a specific
// set of forces. This force will be applied directly to the agent velocity
// during the simulation tick.
type Steer func(a agent.RO) vector.V
