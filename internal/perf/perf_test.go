package perf

import (
	"fmt"
	"testing"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/contrib/arrival"
	"github.com/downflux/go-boids/constraint/contrib/avoidance"
	"github.com/downflux/go-boids/constraint/contrib/seek"
	"github.com/downflux/go-boids/constraint/utils"
	"github.com/downflux/go-boids/constraint/utils/mock"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-database/flags/move"
	"github.com/downflux/go-geometry/2d/hyperrectangle"
	"github.com/downflux/go-geometry/2d/vector"

	magent "github.com/downflux/go-database/agent/mock"
	mfeature "github.com/downflux/go-database/feature/mock"
)

func BenchmarkSteer(b *testing.B) {
	type config struct {
		name string
		s    constraint.Steer
	}

	configs := []config{
		func() config {
			const n = 10
			var steerings []constraint.Steer
			for i := 0; i < n; i++ {
				steerings = append(steerings, mock.M(vector.V{float64(i), float64(i)}))
			}

			return config{
				name: fmt.Sprintf("Sum/N=%v", n),
				s:    utils.Sum(steerings),
			}
		}(),
		{
			name: "Steer",
			s:    utils.Steer(mock.M(vector.V{0, 0})),
		},
		{
			name: "Scale",
			s:    utils.Scale(10, mock.M(vector.V{2, 9})),
		},
		{
			name: "Arrival/SLSDO",
			s:    arrival.SLSDO(vector.V{1, 100}, 10),
		},
		{
			name: "Seek/SLSDO",
			s:    seek.SLSDO(vector.V{1, 100}),
		},
		{
			name: "Avoidance/SLSDO",
			s: avoidance.SLSDO(magent.New(0, agent.O{
				Mass:     100,
				Radius:   10,
				Position: vector.V{100, 100},
			})),
		},
		{
			name: "Avoidance/SLSDOFeature",
			s: avoidance.SLSDOFeature(mfeature.New(0, feature.O{
				AABB: *hyperrectangle.New(vector.V{10, 10}, vector.V{100, 100}),
			})),
		},
	}

	for _, c := range configs {
		b.Run(c.name, func(b *testing.B) {
			a := magent.New(0, agent.O{
				Mass:   10,
				Radius: 5,
				Move:   ^move.FNone,
			})
			for i := 0; i < b.N; i++ {
				c.s(a)
			}
		})
	}
}
