package agent

import (
	"testing"
	"time"

	"github.com/downflux/go-geometry/epsilon"
)

func TestTau(t *testing.T) {
	configs := []struct {
		name     string
		duration time.Duration
		want     float64
	}{
		{
			name:     "Second",
			duration: time.Second,
			want:     1,
		},
		{
			name:     "60Hz",
			duration: 16670 * time.Microsecond,
			want:     1.0 / 60.0,
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := Tau(c.duration); !epsilon.Absolute(1e-5).Within(got, c.want) {
				t.Errorf("Tau() = %v, want = %v", got, c.want)
			}
		})
	}
}
