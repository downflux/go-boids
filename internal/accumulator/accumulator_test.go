package accumulator

import (
	//	"math"
	"testing"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/2d/vector"

	e "github.com/downflux/go-geometry/epsilon"
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
			name: "Magnitude/Within/Torque",
			a: New(
				D{
					Force:  1,
					Torque: 10,
				},
				1,

				// Agent is directed towards the +X axis.
				*polar.New(1, 0),
			),
			force: *vector.New(0, 1),
			want:  *vector.New(0, 1),
			succ:  true,
		},
		{
			name: "Magnitude/Truncated/Torque",
			a: New(
				D{
					Force:  1,
					Torque: 0.5, // rF * sin(𝜃)
				},
				1,

				*polar.New(1, 0),
			),
			force: *vector.New(0, 1),
			want:  *vector.New(0, 0.5),
			succ:  false,
		},
		{
			name: "Magnitude/Within/Force",
			a: New(
				D{
					Force:  10,
					Torque: 1,
				},
				5,
				*polar.New(1, 0),
			),
			force: *vector.New(1, 0),
			want:  *vector.New(1, 0),
			succ:  true,
		},
		{
			name: "Magnitude/Truncated/Force",
			a: New(
				D{
					Force:  1,
					Torque: 1,
				},
				5,

				// Agent is directed towards the +X axis.
				*polar.New(1, 0),
			),
			force: *vector.New(10, 0),
			want:  *vector.New(1, 0),
			succ:  false,
		},
		{
			name: "Magnitude/Truncated/Force/Brake",
			a: New(
				D{
					Force:  1,
					Torque: 1,
				},
				5,

				// Agent is directed towards the +X axis.
				*polar.New(1, 0),
			),
			force: *vector.New(-10, 0),
			want:  *vector.New(-1, 0),
			succ:  false,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := c.a.Add(c.force); !vector.WithinEpsilon(got, c.want, e.Absolute(1e-5)) || ok != c.succ {
				t.Errorf("Add() = %v, %v, want = %v, %v", got, ok, c.want, c.succ)
			}
		})
	}
}
