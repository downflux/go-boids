package boids

import (
	"fmt"
	"time"

	"github.com/downflux/go-boids/x/constraint"
	"github.com/downflux/go-boids/x/constraint/contrib/clamped"
	"github.com/downflux/go-boids/x/constraint/contrib/collision"
	"github.com/downflux/go-boids/x/constraint/contrib/seek"
	"github.com/downflux/go-boids/x/constraint/weighted"
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
	t := float64(d) / float64(time.Second)
	t = 2

	results := make([]result, 0, 256)
	for a := range b.db.ListAgents() {
		results = append(results, result{
			agent: a,
			targetVelocity: clamped.Clamped(
				weighted.GenerateWeightedAverage(
					[]constraint.Accelerator{
						collision.Collision(
							b.db,
							2*a.Radius()+vector.Magnitude(vector.Scale(t, a.Velocity())),
						),
						seek.Seek,
					}, []float64{
						1,
						3,
					}))(a),
		})
	}

	for _, r := range results {
		fmt.Printf("DEBUG: tv(%v) = %v, new = %v\n", r.agent.ID(), r.agent.TargetVelocity(), r.targetVelocity)
		b.db.SetAgentTargetVelocity(r.agent.ID(), r.targetVelocity)
	}
}

type result struct {
	agent          agent.RO
	targetVelocity vector.V
}
