// Package kd provides a shim to lift and downcast caller K-D tree instances to
// the correct type to be consumed by Boid.
//
// The alternative is using generics to enforce types, but after
// experimentation, this syntax is very unwieldy.
package kd

import (
	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
)

// P defines the data that is stored in the Boid K-D tree. Callers to Boid need
// to implement their K-D trees with this point interface instead of the base
// point.P interface.
type P interface {
	point.P

	Agent() agent.A
}

type T kd.T

// Lift ensures the base K-D tree is explicitly casted as a Boid K-D tree
// instead.
func Lift(t *kd.T) *T {
	lt := T(*t)
	return &lt
}

// Downcast transforms the Boid K-D tree back into the base K-D tree.
//
// Note that this function does not normally need to be called -- the Boid
// caller should have separately kept track of the base K-D tree, and only call
// Lift to pass the tree into boid.Step. The underlying tree data has not
// changed.
func Downcast(t *T) *kd.T {
	dt := kd.T(*t)
	return &dt
}

func (t *T) Balance() { Downcast(t).Balance() }

func Agents(ps []P) []agent.A {
	agents := make([]agent.A, 0, len(ps))
	for _, p := range ps {
		agents = append(agents, p.Agent())
	}
	return agents
}

func transform(ps []point.P) []P {
	data := make([]P, 0, len(ps))
	for _, p := range ps {
		data = append(data, p.(P))
	}
	return data
}

func Filter(t *T, r hyperrectangle.R, f func(p P) bool) ([]P, error) {
	ps, err := kd.Filter(Downcast(t), r, func(p point.P) bool { return f(p.(P)) })
	if err != nil {
		return nil, err
	}

	return transform(ps), nil
}

func RadialFilter(t *T, c hypersphere.C, f func(p P) bool) ([]P, error) {
	ps, err := kd.RadialFilter(Downcast(t), c, func(p point.P) bool { return f(p.(P)) })
	if err != nil {
		return nil, err
	}

	return transform(ps), nil
}

func KNN(t *T, q vector.V, k int) ([]P, error) {
	ps, err := kd.KNN(Downcast(t), q, k)
	if err != nil {
		return nil, err
	}

	return transform(ps), nil
}

func Data(t *T) []P { return transform(kd.Data(Downcast(t))) }
