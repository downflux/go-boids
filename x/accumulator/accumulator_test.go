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
		name  string
		a     *A
		force vector.V
		want  vector.V
		succ  bool
	}{
		{
			name: "Angular/Within",
			a: New(
				/* limit = */ *polar.New(2, 10),
				// Agent is directed towards the +X axis.
				/* heading = */ *polar.New(1, 0),
			),
			force: *vector.New(0, 1),
			want:  *vector.New(0, 1),
			succ:  true,
		},
		{
			name: "Angular/Within/Clockwise",
			a: New(
				*polar.New(2, 10),
				*polar.New(1, 0),
			),
			force: *vector.New(0, -1),
			want:  *vector.New(0, -1),
			succ:  true,
		},
		{
			name: "Angular/Within/Brake",
			a: New(
				*polar.New(2, 10),
				*polar.New(1, 0),
			),
			force: *vector.New(-1, 0),
			want:  *vector.New(-1, 0),
			succ:  true,
		},
		{
			name: "Angular/Truncated",
			a: New(
				*polar.New(2, math.Pi/6),
				*polar.New(1, 0),
			),
			force: *vector.New(0, 1),
			want:  *vector.New(math.Sqrt(3)/2.0, 0.5),
			succ:  false,
		},
		{
			name: "Angular/Truncated/Clockwise",
			a: New(
				*polar.New(2, math.Pi/6),
				*polar.New(1, 0),
			),
			force: *vector.New(0, -1),
			want:  *vector.New(math.Sqrt(3)/2.0, -0.5),
			succ:  false,
		},
		{
			name: "Magnitude/Within",
			a: New(
				*polar.New(10, 1),
				*polar.New(1, 0),
			),
			force: *vector.New(1, 0),
			want:  *vector.New(1, 0),
			succ:  true,
		},
		{
			name: "Magnitude/Within/Brake",
			a: New(
				*polar.New(10, 1),
				*polar.New(1, 0),
			),
			force: *vector.New(-1, 0),
			want:  *vector.New(-1, 0),
			succ:  true,
		},
		{
			name: "Magnitude/Truncated",
			a: New(
				*polar.New(10, 1),
				*polar.New(1, 0),
			),
			force: *vector.New(11, 0),
			want:  *vector.New(10, 0),
			succ:  false,
		},
		{
			name: "Magnitude/Truncated/Brake",
			a: New(
				*polar.New(10, 1),
				*polar.New(1, 0),
			),
			force: *vector.New(-11, 0),
			want:  *vector.New(-10, 0),
			succ:  false,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := c.a.Add(c.force); !vector.WithinEpsilon(got, c.want, epsilon.Absolute(1e-5)) || ok != c.succ {
				t.Errorf("Add() = %v, %v, want = %v, %v", got, ok, c.want, c.succ)
			}
		})
	}
}
