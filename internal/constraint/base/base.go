package base

import (
	"container/heap"
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type q []constraint.C

func (q *q) Len() int           { return len(*q) }
func (q *q) Less(i, j int) bool { return (*q)[i].Priority() < (*q)[j].Priority() }
func (q *q) Swap(i, j int)      { (*q)[i], (*q)[j] = (*q)[j], (*q)[i] }
func (q *q) Push(x any) {
	item := x.(constraint.C)
	*q = append(*q, item)
}
func (q *q) Pop() any {
	p := *q
	n := len(p)
	item := p[n-1]
	p[n-1] = nil
	*q = p[0 : n-1]
	return item
}

type C []constraint.C

func New(constraints []constraint.C) *C {
	c := C(constraints)
	return &c
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	pq := make(q, len(c))
	for _, constraint := range c {
		pq = append(pq, constraint)
	}
	heap.Init(&pq)

	// TODO(minkezhang): Account for a.MaxAcceleration().Theta() as well
	// here.
	var v vector.V
	for v = *vector.New(0, 0); pq.Len() > 0 && vector.Magnitude(v) <= a.MaxAcceleration().R(); {
		v = vector.Add(v, heap.Pop(&pq).(vector.V))
	}

	return vector.Scale(
		math.Min(vector.Magnitude(v), a.MaxAcceleration().R()), vector.Unit(v))
}
