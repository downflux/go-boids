package boids

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-database/flags/size"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

const (
	R = 0.5
)

func rn(min, max float64) float64 { return min + rand.Float64()*(max-min) }
func rv(min, max float64) vector.V {
	return vector.V{
		rn(min, max),
		rn(min, max),
	}
}

func BenchmarkTick(b *testing.B) {
	type config struct {
		name     string
		n        int
		coverage float64
	}

	configs := []config{}
	for _, n := range []int{1e3, 1e4, 1e5} {
		for _, coverage := range []float64{0.01, 0.05, 0.1} {
			configs = append(configs, config{
				name:     fmt.Sprintf("N=%v/ρ=%v", n, coverage),
				n:        n,
				coverage: coverage,
			})
		}
	}

	for _, c := range configs {
		b.Run(c.name, func(b *testing.B) {
			b.StopTimer()
			area := float64(c.n) * math.Pi * R * R / c.coverage
			min := 0.0
			max := math.Sqrt(area)

			db := database.New(database.DefaultO)
			boids := New(db, DefaultO)
			for i := 0; i < c.n; i++ {
				db.InsertAgent(agent.O{
					Radius:             R,
					Mass:               1,
					Position:           rv(min, max),
					TargetPosition:     vector.V{0, 0},
					TargetVelocity:     rv(-1, 1),
					Velocity:           rv(-1, 1),
					MaxVelocity:        60,
					MaxAcceleration:    10,
					MaxAngularVelocity: math.Pi / 4,
					Heading:            polar.V{1, 0},
					Size:               size.FSmall,
				})
			}

			// Add world borders.
			// Add xmin border.
			db.InsertFeature(feature.O{
				Min: vector.V{min - 1, min - 1},
				Max: vector.V{min, max + 1},
			})
			// Add xmax border.
			db.InsertFeature(feature.O{
				Min: vector.V{max, min - 1},
				Max: vector.V{max + 1, max + 1},
			})
			// Add ymin border.
			db.InsertFeature(feature.O{
				Min: vector.V{min, min - 1},
				Max: vector.V{max, min},
			})
			// Add ymax border.
			db.InsertFeature(feature.O{
				Min: vector.V{min, max},
				Max: vector.V{max, max + 1},
			})

			b.StartTimer()
			for i := 0; i < b.N; i++ {
				boids.Tick(33 * time.Millisecond)
			}
		})
	}
}
