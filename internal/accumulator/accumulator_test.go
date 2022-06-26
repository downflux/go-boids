package accumulator

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/vector"

	e "github.com/downflux/go-geometry/epsilon"
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
			name:         "Magnitude/Within",
			a:            New(10),
			acceleration: *vector.New(1, 1),
			want:         *vector.New(1, 1),
			succ:         true,
		},
		{
			name: "Magnitude/Truncated",
			a: &A{
				limit: 1,
			},
			acceleration: *vector.New(10, 10),
			want: vector.Scale(
				math.Sqrt(2)/2,
				*vector.New(1, 1),
			),
			succ: false,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := c.a.Add(c.acceleration); !vector.WithinEpsilon(got, c.want, e.Absolute(1e-5)) || ok != c.succ {
				t.Errorf("Add() = %v, %v, want = %v, %v", got, ok, c.want, c.succ)
			}
		})
	}
}
