package config

import (
	"fmt"
	"testing"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestStep(t *testing.T) {
	configs := []struct {
		name     string
		a        *A
		steering vector.V
		tau      float64

		heading polar.V
		v       vector.V
		p       vector.V
	}{
		{
			name: "Trivial/NoTurn",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, 0),
				},
			},
			steering: *vector.New(1, 0),
			tau:      1,

			heading: *polar.New(1, 0),
			v:       *vector.New(1, 0),
			p:       *vector.New(2, 0),
		},
		{
			name: "Truncate/Magnitude/NoTurn",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, 0),
				},
			},
			steering: *vector.New(100, 0),
			tau:      1,

			heading: *polar.New(1, 0),
			v:       *vector.New(1, 0),
			p:       *vector.New(2, 0),
		},
		{
			name: "Truncate/Magnitude/Reverse",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, 0),
				},
			},
			steering: *vector.New(-100, 0),
			tau:      1,

			heading: *polar.New(1, 0),
			v:       *vector.New(-1, 0),
			p:       *vector.New(0, 0),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			c.a.Step(c.v, c.tau)
			t.Run(fmt.Sprintf("%v/Heading", c.name), func(t *testing.T) {
				if got := c.a.Heading(); !polar.Within(got, c.heading) {
					t.Errorf("Heading() = %v, want = %v", got, c.heading)
				}
			})
			t.Run(fmt.Sprintf("%v/V", c.name), func(t *testing.T) {
				if got := c.a.V(); !vector.Within(got, c.v) {
					t.Errorf("V() = %v, want = %v", got, c.v)
				}
			})
			t.Run(fmt.Sprintf("%v/P", c.name), func(t *testing.T) {
				if got := c.a.P(); !vector.Within(got, c.p) {
					t.Errorf("P() = %v, want = %v", got, c.p)
				}
			})
		})
	}
}
