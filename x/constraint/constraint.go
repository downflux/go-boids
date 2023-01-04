package constraint

import (
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// Accelerator is a function which returns the desired net acceleration for a
// specific set of forces. For seek behavior, the acceleration vector will point
// directly to the target.
type Accelerator func(a agent.RO) vector.V
