package boids

import (
	"time"

	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-boids/x/constraint/clamped"
	"github.com/downflux/go-boids/x/constraint/contrib/avoidance"
	"github.com/downflux/go-boids/x/constraint/contrib/seek"
	"github.com/downflux/go-boids/x/constraint/scale"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	DefaultO = O{
		PoolSize:        24,
		AvoidanceWeight: 5,
		SeekWeight:      1,
	}
)

type O struct {
	PoolSize int

	AvoidanceWeight float64
	SeekWeight      float64
}

type B struct {
	db       *database.DB
	poolSize int

	avoidanceWeight float64
	seekWeight      float64
}

func New(db *database.DB, o O) *B {
	return &B{
		db:       db,
		poolSize: o.PoolSize,

		avoidanceWeight: o.AvoidanceWeight,
		seekWeight:      o.SeekWeight,
	}
}

// TODO(minkezhang): Make concurrent.
func (b *B) Tick(d time.Duration) {
	t := float64(d) / float64(time.Second)
	results := make([]result, 0, 256)
	for a := range b.db.ListAgents() {
		results = append(results, result{
			agent: a,
			acceleration: clamped.Clamped(
				[]constraint.Accelerator{
					scale.Scale(b.avoidanceWeight, avoidance.Avoid(b.db, time.Second)),
					// TODO(minkezhang): Cohesion.
					// TODO(minkezhang): Arrival.
					scale.Scale(b.seekWeight, seek.SLSDO),
				},
				a.MaxAcceleration(),
			)(a),
		})
	}

	for _, r := range results {
		a := vector.Scale(t, r.acceleration)
		v := vector.Add(r.agent.Velocity(), a)
		b.db.SetAgentTargetVelocity(r.agent.ID(), v)
	}
}

type result struct {
	agent        agent.RO
	acceleration vector.V
}
