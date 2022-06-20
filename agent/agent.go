package agent

import (
	"github.com/downflux/go-geometry/2d/vector"
)

type A interface {
	Position() vector.V
	Velocity() vector.V
	Acceleration() vector.V
	Radius() float64
	Mass() float64

	MaxAcceleration() vector.V
}
