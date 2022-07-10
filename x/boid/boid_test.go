package boid

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"runtime"
	"testing"

	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-boids/x/agent"
	"github.com/downflux/go-boids/x/agent/mock"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	bkd "github.com/downflux/go-boids/x/kd"
	v2d "github.com/downflux/go-geometry/2d/vector"
)

const (
	MinN = int(1e3)
	// TODO(minkezhang): Make k-D tree more performant so that we can
	// increase this max N.
	MaxN = int(1e3)
)

var _ bkd.P = p{}

type p struct {
	a agent.RO
}

func (p p) Agent() agent.RO { return p.a }
func (p p) P() vector.V     { return vector.V(p.a.P()) }

func rn(min, max float64) float64 { return min + rand.Float64()*(max-min) }
func rv() v2d.V                   { return *v2d.New(rn(0, 200), rn(0, 200)) }
func rp() polar.V                 { return *polar.New(rn(0, 200), rn(0, 2*math.Pi)) }
func ra(i int) agent.RO {
	a := mock.New(mock.O{
		P:           rv(),
		V:           rv(),
		R:           rn(10, 20),
		Mass:        rn(10, 20),
		Heading:     *polar.New(1, rp().Theta()),
		MaxSpeed:    rn(0, 200),
		MaxNetForce: rn(0, 2000),
	})
	fp, _ := os.Create(os.DevNull)
	a.Log = log.New(fp, "", 0)
	return a
}

func rt(n int) *kd.T {
	// Generating large number of points in tests will mess with data
	// collection figures. We should ignore these allocs.
	runtime.MemProfileRate = 0
	defer func() { runtime.MemProfileRate = 512 * 1024 }()

	ps := make([]point.P, 0, n)
	for i := 0; i < n; i++ {
		ps = append(ps, p{a: ra(i)})
	}
	t, _ := kd.New(ps)
	return t
}

func BenchmarkStep(b *testing.B) {
	type config struct {
		name string
		t    *kd.T
		size int
	}

	configs := []config{}
	for n := MinN; n <= MaxN; n = n * 10 {
		for size := 1; size <= n && size < 128; size = size << 1 {
			configs = append(configs, config{
				name: fmt.Sprintf("PoolSize=%v/N=%v", size, n),
				t:    rt(n),
				size: size,
			})
		}
	}

	for _, c := range configs {
		b.Run(c.name, func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				Step(O{
					PoolSize:  c.size,
					T:         bkd.Lift(c.t),
					Tau:       1e-2,
					MaxRadius: 20,

					CollisionWeight: 50,
					CollisionFilter: func(a agent.RO) bool { return true },
					ArrivalWeight:   6,
				})
			}
		})
	}
}
