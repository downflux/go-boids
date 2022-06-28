package config

import (
	"math"
	"testing"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
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
			name: "Trivial/Speed",
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
			name: "Trivial/Rotate",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, math.Pi),
				},
			},
			steering: *vector.New(0, 1),
			tau:      1,

			heading: *polar.New(1, math.Pi/2),
			v:       *vector.New(0, 1),
			p:       *vector.New(1, 1),
		},
		{
			name: "Truncate/Speed",
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
			name: "Truncate/Rotate",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, math.Pi/4),
				},
			},
			steering: *vector.New(0, 1),
			tau:      1,

			heading: *polar.New(1, math.Pi/4),
			v:       *vector.New(math.Sqrt(2)/2, math.Sqrt(2)/2),
			p:       *vector.New(1+math.Sqrt(2)/2, math.Sqrt(2)/2),
		},
		{
			name: "Truncate/Rotate/Reverse",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, math.Pi/4),
				},
			},
			steering: *vector.New(-1, 0),
			tau:      1,

			heading: *polar.New(1, math.Pi/4),
			v:       *vector.New(-math.Sqrt(2)/2, -math.Sqrt(2)/2),
			p:       *vector.New(1-math.Sqrt(2)/2, -math.Sqrt(2)/2),
		},
		{
			name: "Truncate/Rotate/Reverse/Partial",
			a: &A{
				O: O{
					P:           *vector.New(1, 0),
					V:           *vector.New(0, 0),
					Heading:     *polar.New(1, 0),
					MaxVelocity: *polar.New(1, math.Pi/4),
				},
			},
			steering: *vector.New(-1, 1),
			tau:      1,

			heading: *polar.New(1, -math.Pi/4),
			v:       *vector.New(-math.Sqrt(2)/2, math.Sqrt(2)/2),
			p:       *vector.New(1-math.Sqrt(2)/2, math.Sqrt(2)/2),
		},
		{
			name: "Truncate/Speed/Reverse",
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
			c.a.Step(c.steering, c.tau)
			t.Run("Heading", func(t *testing.T) {
				if got := c.a.Heading(); !polar.Within(got, c.heading) {
					t.Errorf("Heading() = %v, want = %v", got, c.heading)
				}
			})
			t.Run("V", func(t *testing.T) {
				if got := c.a.V(); !vector.WithinEpsilon(got, c.v, epsilon.Absolute(1e-5)) {
					t.Errorf("V() = %v, want = %v", got, c.v)
				}
			})
			t.Run("P", func(t *testing.T) {
				if got := c.a.P(); !vector.WithinEpsilon(got, c.p, epsilon.Absolute(1e-5)) {
					t.Errorf("P() = %v, want = %v", got, c.p)
				}
			})
		})
	}
}
