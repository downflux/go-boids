package boids

import (
	"time"

	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-boids/x/constraint/clamped"
	"github.com/downflux/go-boids/x/constraint/contrib/seek"
	"github.com/downflux/go-boids/x/constraint/contrib/separation"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-geometry/2d/vector"
)

var (
	DefaultO = O{
		PoolSize: 24,
	}
)

type O struct {
	PoolSize int
}

type B struct {
	db       *database.DB
	poolSize int
}

func New(db *database.DB, o O) *B {
	return &B{
		db:       db,
		poolSize: o.PoolSize,
	}
}

func (b *B) Tick(d time.Duration) {
	results := make([]result, 0, 256)
	for a := range b.db.ListAgents() {
		rsep := 2*a.Radius() + vector.Magnitude(a.Velocity())
		results = append(results, result{
			agent: a,
			targetVelocity: clamped.Clamped(
				[]constraint.Accelerator{
					separation.Separation(b.db, rsep),
					seek.Seek,
				}, a.MaxAcceleration(),
			)(a),
		})
		// TODO(minkezhang): ClampAngularVelocity
	}

	for _, r := range results {
		b.db.SetAgentTargetVelocity(r.agent.ID(), r.targetVelocity)
	}
}

type result struct {
	agent          agent.RO
	targetVelocity vector.V
}
