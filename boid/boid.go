package boid

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/kd"
	"github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	T *kd.T

	Tau float64
}

type Mutation struct {
	Agent        agent.A
	Acceleration vector.V
}

func Step(o O) []Mutation {

	return nil
}
