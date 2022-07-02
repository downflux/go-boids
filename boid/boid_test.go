package boid

import (
	"fmt"
	"math"
	"math/rand"
	"runtime"
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/internal/geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"

	mock "github.com/downflux/go-boids/agent/mock"
	bkd "github.com/downflux/go-boids/kd"
	v2d "github.com/downflux/go-geometry/2d/vector"
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
	return mock.New(mock.O{
		ID:           agent.ID(fmt.Sprintf("%v", i)),
		P:            rv(),
		V:            rv(),
		R:            rn(10, 20),
		Mass:         rn(10, 20),
		Goal:         rv(),
		Heading:      *polar.New(1, rp().Theta()),
		MaxVelocity:  rp(),
		MaxNetForce:  rn(0, 2000),
		MaxNetTorque: rn(0, 20000),
	})
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
	for n := 1000; n < 1000000; n = n * 10 {
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
					PoolSize: c.size,
					T:        bkd.Lift(c.t),
					Tau:      1e-2,
					MaxRadius: 20,

					CollisionWeight: 50,
					CollisionFilter: func(a agent.RO) bool { return true },
					ArrivalWeight: 6,
				})
			}
		})
	}
}
