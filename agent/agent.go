package agent

import (
	"github.com/downflux/go-geometry/2d/vector"
)

type RW interface {
	A
	WO
}

type WO interface {
	SetP(v vector.V)
	SetV(v vector.V)
	SetHeading(v vector.V)
}

// TODO(minkezhang): Rename RO.
type A interface {
	P() vector.V
	V() vector.V
	R() float64
	Mass() float64
	Heading() vector.V

	Goal() vector.V

	MaxSpeed() float64
	MaxNetForce() float64
}
