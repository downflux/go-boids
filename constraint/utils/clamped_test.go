package utils

import (
	"testing"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/utils/mock"
	"github.com/downflux/go-geometry/2d/vector"

	magent "github.com/downflux/go-database/agent/mock"
)

func TestClamped(t *testing.T) {
	type config struct {
		name      string
		steerings []constraint.Steer
		limit     float64
		want      vector.V
	}

	configs := []config{
		{
			name:      "Empty",
			steerings: nil,
			limit:     0,
			want:      vector.V{0, 0},
		},
		{
			name: "Trivial",
			steerings: []constraint.Steer{
				mock.M(vector.V{1, 1}),
			},
			limit: 10,
			want:  vector.V{1, 1},
		},
		{
			name: "TooLarge",
			steerings: []constraint.Steer{
				mock.M(vector.V{100, 0}),
			},
			limit: 10,
			want:  vector.V{10, 0},
		},
		{
			name: "Truncate",
			steerings: []constraint.Steer{
				mock.M(vector.V{-1, 0}),
				mock.M(vector.V{10, 0}),
			},
			limit: 10,
			want:  vector.V{8, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := Clamped(c.steerings, c.limit)(&magent.A{})
			if !vector.Within(got, c.want) {
				t.Errorf("Clamped() = %v, want = %v", got, c.want)
			}
		})
	}
}
