package base

import (
	//	"container/heap"
	"math"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"
)

var _ constraint.C = C{}

type item struct {
	value constraint.C
	index int
}

type q []*item

func (q q) Len() int           { return len(q) }
func (q q) Less(i, j int) bool { return q[i].value.Priority() < q[j].value.Priority() }
func (q q) Swap(i, j int) {
	q[i], q[j] = q[j], q[i]
	q[i].index = i
	q[j].index = j
}
func (q *q) Push(x any) {
	i := &item{
		index: q.Len(),
		value: x.(constraint.C),
	}
	*q = append(*q, i)
}
func (q *q) Pop() any {
	p := *q
	n := len(p)
	item := p[n-1]
	p[n-1] = nil
	item.index = -1
	*q = p[0 : n-1]
	return item.value
}

type C []constraint.C

func New(constraints []constraint.C) *C {
	c := C(constraints)
	return &c
}

func (c C) Priority() constraint.P { return 0 }

func (c C) A(a agent.A) vector.V {
	/*
		pq := make(q, len(c))
		for i, constraint := range c {
			pq = append(pq, &item{
				index: i,
				value: constraint,
			})
		}
		heap.Init(&pq)
	*/

	// TODO(minkezhang): Account for a.MaxAcceleration().Theta() as well
	// here.
	v := *vector.New(0, 0)
	var s float64
	for _, constraint := range c {
		s += vector.Magnitude(v)
		v = vector.Add(v, constraint.A(a))
		// TODO(minkezhang): Truncate this last value vs. adding it in
		// directly.
		/*
			if s >= a.MaxAcceleration().R() {
				break
			}
		*/
	}
	/*
		for v = *vector.New(0, 0); pq.Len() > 0 && vector.Magnitude(v) <= a.MaxAcceleration().R(); {
			v = vector.Add(v, heap.Pop(&pq).(constraint.C).A(a))
		}
	*/
	if vector.Within(*vector.New(0, 0), v) {
		return *vector.New(0, 0)
	}

	return vector.Scale(
		math.Min(vector.Magnitude(v), a.MaxAcceleration().R()), vector.Unit(v))
}
