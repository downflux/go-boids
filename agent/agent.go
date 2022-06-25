package agent

import (
	"github.com/downflux/go-boids/internal/geometry/2d/vector/cylindrical"
	"github.com/downflux/go-geometry/2d/vector"
)

type A interface {
	P() vector.V
	V() vector.V
	R() float64

	Goal() vector.V

	MaxSpeed() float64
	MaxAcceleration() cylindrical.V
}
