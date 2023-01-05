package boids

import (
	"time"

	"github.com/downflux/go-boids/x/constraint/contrib/collision"
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

	results := make([]result, 0, 256)
	for a := range b.db.ListAgents() {
		results = append(results, result{
			agent: a,
			targetVelocity: collision.Collision(
				b.db,
				vector.Magnitude(vector.Scale(t, a.Velocity())),
			)(a),
		})
	}

	for _, r := range results {
		b.db.SetAgentTargetVelocity(r.agent.ID(), r.targetVelocity)
	}
}

type result struct {
	agent          agent.RO
	targetVelocity vector.V
}
