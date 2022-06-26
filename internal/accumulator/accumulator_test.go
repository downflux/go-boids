package accumulator

import (
	"math"
	"testing"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
)

func TestAdd(t *testing.T) {
	configs := []struct {
		name         string
		a            *A
		acceleration vector.V
		want         vector.V
		succ         bool
	}{
		{
			name: "Magnitude/Within",
			a: &A{
				limit:       *polar.New(10, 0),
				accumulator: *polar.New(0, 0),
				velocity:    *polar.New(1, math.Pi/4),
			},
			acceleration: *vector.New(1, 1),
			want:         *vector.New(1, 1),
			succ:         true,
		},
		{
			name: "Magnitude/Truncated",
			a: &A{
				limit:       *polar.New(2, 0),
				accumulator: *polar.New(0, 0),
				velocity:    *polar.New(1, math.Pi/4),
			},
			acceleration: *vector.New(10, 10),
			want: vector.Scale(
				math.Sqrt(2),
				*vector.New(1, 1),
			),
			succ: false,
		},
		{
			name: "Theta/Within",
			a: &A{
				limit:       *polar.New(10, math.Pi),
				accumulator: *polar.New(0, 0),
				velocity:    *polar.New(1, 0),
			},
			acceleration: *vector.New(1, 1),
			want:         *vector.New(1, 1),
			succ:         true,
		},
		{
			name: "Theta/Truncated/Slight",
			a: &A{
				limit:       *polar.New(10, math.Pi/2),
				accumulator: *polar.New(0, math.Pi/4),
				velocity:    *polar.New(1, 0),
			},
			acceleration: *vector.New(-1, 0),
			want:         *vector.New(-0.5, -0.5),
			succ:         false,
		},
		{
			name: "Theta/Truncated/Total",
			a: &A{
				limit:       *polar.New(10, 0),
				accumulator: *polar.New(0, 0),
				velocity:    *polar.New(1, 0),
			},
			acceleration: *vector.New(-1, 9),
			want:         *vector.New(-1, 0),
			succ:         false,
		},
		{
			name: "Theta/Truncated/Normal",
			a: &A{
				limit:       *polar.New(10, math.Pi),
				accumulator: *polar.New(0, math.Pi/2),
				velocity:    *polar.New(1, 0),
			},
			acceleration: *vector.New(-1, 0),
			want:         *vector.New(0, 0),
			succ:         false,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := c.a.Add(c.acceleration); !vector.WithinEpsilon(got, c.want, epsilon.Absolute(1e-5)) || ok != c.succ {
				t.Errorf("Add() = %v, %v, want = %v, %v", got, ok, c.want, c.succ)
			}
		})
	}
}
