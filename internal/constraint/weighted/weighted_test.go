package weighted

import (
	"math"
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-boids/constraint"
	"github.com/downflux/go-geometry/2d/vector"

	mock "github.com/downflux/go-boids/agent/mock"
)

var _ constraint.C = M{}

type M vector.V

func (m M) Force(a agent.RO) vector.V { return vector.V(m) }

func TestForce(t *testing.T) {
	configs := []struct {
		name string
		c    C
		a    agent.RO
		want vector.V
	}{
		{
			name: "Trivial",
			c:    *New(nil, nil),
			a:    nil,
			want: *vector.New(0, 0),
		},
		{
			name: "Single/NonZero",
			c: *New([]constraint.C{
				M(*vector.New(10, 10)),
			}, []float64{100}),
			a: mock.New(mock.O{
				MaxNetForce: math.Sqrt(2),
				Mass:        10,
			}),
			want: *vector.New(1, 1),
		},
		{
			name: "Zero",
			c: *New([]constraint.C{
				M(*vector.New(0, 0)),
			}, []float64{100}),
			a:    mock.New(mock.O{MaxNetForce: math.Sqrt(2)}),
			want: *vector.New(0, 0),
		},
		{
			name: "Multiple/NonZero",
			c: *New([]constraint.C{
				M(*vector.New(10, 0)),
				M(*vector.New(0, 5)),
			}, []float64{
				1,
				2,
			}),
			a: mock.New(mock.O{
				MaxNetForce: math.Sqrt(2),
				Mass:        10,
			}),
			want: *vector.New(1, 1),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.c.Force(c.a); !vector.Within(got, c.want) {
				t.Errorf("Force() = %v, want = %v", got, c.want)
			}
		})
	}
}
