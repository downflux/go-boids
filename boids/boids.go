package boids

import (
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

		// FlockingWeight is a metaparameter which touches the
		// alignment, separation, and cohesion weights while keeping
		// their relative strengths.
		FlockingWeight: 1,

		AlignmentWeight:  0.5,
		AlignmentHorizon: 2,

		SeparationWeight:  0.5,
		SeparationHorizon: 2,

		CohesionWeight:  0.4,
		CohesionHorizon: 5,
	}
)

type O struct {
	PoolSize int

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
	db       *database.DB
	poolSize int

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
	return &B{
		db:       db,
		poolSize: o.PoolSize,

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

// TODO(minkezhang): Make concurrent.
func (b *B) Tick(d time.Duration) {
	t := float64(d) / float64(time.Second)

	results := make([]result, 0, 256)
	for a := range b.db.ListAgents() {
		arrivalR := b.arrivalHorizon * a.Radius()
		alignmentR := b.alignmentHorizon * a.Radius()
		avoidanceR := a.Radius() + b.avoidanceHorizon*vector.Magnitude(a.Velocity())
		separationR := b.separationHorizon * a.Radius()
		cohesionR := b.cohesionHorizon * a.Radius()

		// First term in this clamped velocity allows collision
		// avoidance to take precedent.
		collision := utils.Scale(b.avoidanceWeight, avoidance.Avoid(b.db, avoidanceR))

		// Second term in this clamped velocity allows contribution from
		// all sources.
		weighted := []constraint.Accelerator{
			utils.Scale(b.seekWeight, seek.SLSDO(a.TargetPosition())),
			// TODO(minkezhang): Add agent.Stable() to indicate the
			// agent should stop.
			utils.Scale(b.arrivalWeight, arrival.SLSDO(a.TargetPosition(), arrivalR)),
			utils.Scale(b.alignmentWeight, alignment.Align(b.db, alignmentR)),
			utils.Scale(b.separationWeight, separation.Separation(b.db, separationR)),
			utils.Scale(-b.cohesionWeight, separation.Separation(b.db, cohesionR)),
		}

		results = append(results, result{
			agent: a,
			steer: utils.Clamped(
				[]constraint.Accelerator{
					collision,
					utils.Sum(weighted),
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
