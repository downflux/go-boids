package boids

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/contrib/alignment"
	"github.com/downflux/go-boids/constraint/contrib/arrival"
	"github.com/downflux/go-boids/constraint/contrib/avoidance"
	"github.com/downflux/go-boids/constraint/contrib/seek"
	"github.com/downflux/go-boids/constraint/contrib/separation"
	"github.com/downflux/go-boids/constraint/utils"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/database/cache"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

var (
	DefaultO = O{
		PoolSize: 24,

		// Optionally instruct the simulation to do a single BroadPhase
		// query per tick. This query will use the largest radius
		// available. This is used when the simulation is expected to
		// incur a large BVH query penalty.
		EnableCache: false,

		AvoidanceWeight:  5,
		AvoidanceHorizon: 1.5,

		SeekWeight: 1,

		ArrivalWeight:  1,
		ArrivalHorizon: 3,

		// FlockingWeight is a metaparameter which touches the
		// alignment, separation, and cohesion weights while keeping
		// their relative strengths.
		FlockingWeight: 0.3,

		AlignmentWeight:  1,
		AlignmentHorizon: 4,

		SeparationWeight:  2.5,
		SeparationHorizon: 2.5,

		CohesionWeight:  1,
		CohesionHorizon: 5,
	}
)

type O struct {
	PoolSize    int
	EnableCache bool

	AvoidanceWeight   float64
	AvoidanceHorizon  float64
	SeekWeight        float64
	ArrivalWeight     float64
	ArrivalHorizon    float64
	AlignmentWeight   float64
	AlignmentHorizon  float64
	SeparationWeight  float64
	SeparationHorizon float64
	CohesionWeight    float64
	CohesionHorizon   float64
	FlockingWeight    float64
}

type B struct {
	db          *database.DB
	poolSize    int
	enableCache bool

	avoidanceWeight   float64
	avoidanceHorizon  float64
	seekWeight        float64
	arrivalWeight     float64
	arrivalHorizon    float64
	alignmentWeight   float64
	alignmentHorizon  float64
	separationWeight  float64
	separationHorizon float64
	cohesionWeight    float64
	cohesionHorizon   float64
}

func New(db *database.DB, o O) *B {
	if o.PoolSize < 1 {
		panic(fmt.Sprintf("PoolSize specified %v is smaller than the minimum value of 1", o.PoolSize))
	}

	return &B{
		db:          db,
		poolSize:    o.PoolSize,
		enableCache: o.EnableCache,

		avoidanceWeight:   o.AvoidanceWeight,
		avoidanceHorizon:  o.AvoidanceHorizon,
		seekWeight:        o.SeekWeight,
		arrivalWeight:     o.ArrivalWeight,
		arrivalHorizon:    o.ArrivalHorizon,
		alignmentWeight:   o.FlockingWeight * o.AlignmentWeight,
		alignmentHorizon:  o.AlignmentHorizon,
		separationWeight:  o.FlockingWeight * o.SeparationWeight,
		separationHorizon: o.SeparationHorizon,
		cohesionWeight:    o.FlockingWeight * o.CohesionWeight,
		cohesionHorizon:   o.CohesionHorizon,
	}
}

func (b *B) cache(a agent.RO, rs ...float64) database.RO {
	r := math.Inf(-1)
	for _, x := range rs {
		if x > r {
			r = x
		}
	}

	p := a.Position()
	aabb := hyperrectangle.New(
		vnd.V{
			p.X() - r,
			p.Y() - r,
		},
		vnd.V{
			p.X() + r,
			p.Y() + r,
		},
	)

	return cache.New(cache.O{
		Agents:   b.db.QueryAgents(*aabb, func(a agent.RO) bool { return true }),
		Features: b.db.QueryFeatures(*aabb, func(f feature.RO) bool { return true }),
	})
}

func (b *B) generate() []result {
	agents := b.db.ListAgents()
	ch := make(chan result, 1024)

	go func(ch chan<- result) {
		var wg sync.WaitGroup
		wg.Add(b.poolSize)
		for i := 0; i < b.poolSize; i++ {
			go func(ch chan<- result) {
				defer wg.Done()
				for a := range agents {
					arrivalR := b.arrivalHorizon * a.Radius()
					alignmentR := b.alignmentHorizon * a.Radius()
					avoidanceR := a.Radius() + b.avoidanceHorizon*vector.Magnitude(a.Velocity())
					separationR := b.separationHorizon * a.Radius()
					cohesionR := b.cohesionHorizon * a.Radius()

					var c database.RO = b.db
					if b.enableCache {
						c = b.cache(
							a,
							arrivalR,
							alignmentR,
							avoidanceR,
							separationR,
							cohesionR,
						)
					}

					// First term in this clamped velocity allows collision
					// avoidance to take precedent.
					collision := utils.Scale(b.avoidanceWeight, avoidance.Avoid(c, avoidanceR))

					// Second term in this clamped velocity allows contribution from
					// all sources.
					weighted := []constraint.Steer{
						utils.Scale(b.seekWeight, seek.SLSDO(a.TargetPosition())),
						utils.Scale(b.arrivalWeight, arrival.SLSDO(a.TargetPosition(), arrivalR)),
						utils.Scale(b.alignmentWeight, alignment.Align(c, alignmentR)),
						utils.Scale(b.separationWeight, separation.Separation(c, separationR)),
						utils.Scale(-b.cohesionWeight, separation.Separation(c, cohesionR)),
					}

					ch <- result{
						agent: a,
						steer: utils.Clamped(
							[]constraint.Steer{
								collision,
								utils.Sum(weighted),
							},
							a.MaxAcceleration(),
						)(a),
					}
				}
			}(ch)
		}
		wg.Wait()
		close(ch)
	}(ch)

	results := make([]result, 0, 1024)
	for r := range ch {
		results = append(results, r)
	}
	return results
}

func (b *B) Tick(d time.Duration) {
	t := float64(d) / float64(time.Second)

	// N.B.: This can also be make concurrent, but these are straightforward
	// property setters and not worth the extra overhead of allocating e.g.
	// channels.
	for _, r := range b.generate() {
		a := vector.Scale(t, r.steer)
		v := vector.Add(r.agent.Velocity(), a)
		b.db.SetAgentTargetVelocity(r.agent.ID(), v)
	}
}

type result struct {
	agent agent.RO
	steer vector.V
}
