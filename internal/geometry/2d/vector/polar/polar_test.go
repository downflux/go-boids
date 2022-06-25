package polar

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
)

func TestPolar(t *testing.T) {
	configs := []struct {
		name string
		v    vector.V
		want V
	}{
		{
			name: "R=0",
			v:    *vector.New(0, 0),
			want: *New(0, 0),
		},
		{
			name: "Theta/Vertical/PositiveY",
			v:    *vector.New(0, 1),
			want: *New(1, math.Pi/2),
		},
		{
			name: "Theta/Vertical/NegativeY",
			v:    *vector.New(0, -1),
			want: *New(1, -math.Pi/2),
		},
		{
			name: "Theta/Q1",
			v:    *vector.New(1, 1),
			want: *New(math.Sqrt(2), math.Pi/4),
		},
		{
			name: "Theta/Q2",
			v:    *vector.New(-1, 1),
			want: *New(math.Sqrt(2), 3*math.Pi/4),
		},
		{
			name: "Theta/Q3",
			v:    *vector.New(-1, -1),
			want: *New(math.Sqrt(2), 5*math.Pi/4),
		},
		{
			name: "Theta/Q4",
			v:    *vector.New(1, -1),
			want: *New(math.Sqrt(2), -math.Pi/4),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := Polar(c.v); !Within(got, c.want) {
				t.Errorf("Polar() = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestWithin(t *testing.T) {
	configs := []struct {
		name string
		u    V
		v    V
		want bool
	}{
		{
			name: "Within/Trivial",
			u:    *New(1, 1),
			v:    *New(1, 1),
			want: true,
		},
		{
			name: "Within/Trivial/False",
			u:    *New(1, 2),
			v:    *New(1, 1),
			want: false,
		},
		{
			name: "Within/Rotate",
			u:    *New(1, 1),
			v:    *New(1, 1+2*math.Pi),
			want: true,
		},
		{
			name: "Within/Rotate/False",
			u:    *New(1, 1),
			v:    *New(1, 1+math.Pi),
			want: false,
		},
		{
			name: "Within/Rotate/R=0",
			u:    *New(0, 1),
			v:    *New(0, 2),
			want: true,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := Within(c.u, c.v); got != c.want {
				t.Errorf("Within() = %v, want = %v", got, c.want)
			}
		})
	}
}
