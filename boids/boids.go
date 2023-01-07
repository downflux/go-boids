package boids

import (
	"math"
	"time"

	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-boids/constraint/contrib/alignment"
	"github.com/downflux/go-boids/constraint/contrib/arrival"
	"github.com/downflux/go-boids/constraint/contrib/avoidance"
	"github.com/downflux/go-boids/constraint/contrib/seek"
	"github.com/downflux/go-boids/constraint/utils"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	DefaultO = O{
		PoolSize: 24,

		AvoidanceWeight:  5,
		AvoidanceHorizon: 1.5,

		// TODO(minkezhang): Add an agent.Mode() which conditionally
		// uses selective steering behaviors.
		SeekWeight: 0,

		ArrivalWeight:  1,
		ArrivalHorizon: 3,

		AlignmentWeight:  0.5,
		AlignmentHorizon: 2,
	}
)

type O struct {
	PoolSize int

	AvoidanceWeight  float64
	AvoidanceHorizon float64
	SeekWeight       float64
	ArrivalWeight    float64
	ArrivalHorizon   float64
	AlignmentWeight  float64
	AlignmentHorizon float64
}

type B struct {
	db       *database.DB
	poolSize int

	avoidanceWeight  float64
	avoidanceHorizon float64
	seekWeight       float64
	arrivalWeight    float64
	arrivalHorizon   float64
	alignmentWeight  float64
	alignmentHorizon float64
}

func New(db *database.DB, o O) *B {
	return &B{
		db:       db,
		poolSize: o.PoolSize,

		avoidanceWeight:  o.AvoidanceWeight,
		avoidanceHorizon: o.AvoidanceHorizon,
		seekWeight:       o.SeekWeight,
		arrivalWeight:    o.ArrivalWeight,
		arrivalHorizon:   o.ArrivalHorizon,
		alignmentWeight:  o.AlignmentWeight,
		alignmentHorizon: o.AlignmentHorizon,
	}
}

// TODO(minkezhang): Make concurrent.
func (b *B) Tick(d time.Duration) {
	t := float64(d) / float64(time.Second)

	results := make([]result, 0, 256)
	for a := range b.db.ListAgents() {
		arrivalR := b.arrivalHorizon * a.Radius()
		alignmentR := b.alignmentHorizon * a.Radius()
		avoidanceR := a.Radius() + b.avoidanceHorizon*vector.Magnitude(a.Velocity())

		// First term in this clamped velocity allows collision
		// avoidance to take precedent.
		collision := utils.Scale(b.avoidanceWeight, avoidance.Avoid(b.db, avoidanceR))

		// Second term in this clamped velocity allows contribution from
		// all sources.
		weighted := []constraint.Accelerator{
			// TODO(minkezhang): Cohesion.
			// TODO(minkezhang): Separation.
			utils.Scale(b.seekWeight, seek.SLSDO(a.TargetPosition())),
			// TODO(minkezhang): Add agent.Stable() to indicate the
			// agent should stop.
			utils.Scale(b.arrivalWeight, arrival.SLSDO(a.TargetPosition(), arrivalR)),
			utils.Scale(b.alignmentWeight, alignment.Align(b.db, alignmentR)),
		}

		results = append(results, result{
			agent: a,
			steer: utils.Clamped(
				[]constraint.Accelerator{
					collision,
					utils.Clamped(weighted, math.Inf(1)),
				},
				a.MaxAcceleration(),
			)(a),
		})
	}

	for _, r := range results {
		a := vector.Scale(t, r.steer)
		v := vector.Add(r.agent.Velocity(), a)
		b.db.SetAgentTargetVelocity(r.agent.ID(), v)
	}
}

type result struct {
	agent agent.RO
	steer vector.V
}
