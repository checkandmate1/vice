// aviation/intent_test.go
// Copyright(c) 2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package aviation

import (
	"strings"
	"testing"

	"github.com/mmp/vice/rand"
)

func renderIntentForTest(intent CommandIntent, seed uint64) string {
	if DB == nil {
		DB = &StaticDatabase{
			Navaids:  map[string]Navaid{},
			Airports: map[string]FAAAirport{},
		}
	}

	r := &rand.Rand{PCG32: rand.NewPCG32()}
	r.Seed(seed)
	return strings.ToLower(RenderIntents([]CommandIntent{intent}, r).Written(r))
}

func assertContainsAny(t *testing.T, readback string, values ...string) {
	t.Helper()

	for _, s := range values {
		if strings.Contains(readback, s) {
			return
		}
	}
	t.Fatalf("readback %q does not contain any of %v", readback, values)
}

func TestSpeedRestrictionReadbackIncludesQualifier(t *testing.T) {
	for _, test := range []struct {
		name       string
		intent     SpeedIntent
		qualifiers []string
	}{
		{
			name:       "bare or greater",
			intent:     SpeedIntent{Speed: 250, Type: SpeedAtOrAbove},
			qualifiers: []string{"or greater", "or above"},
		},
		{
			name:       "bare or less",
			intent:     SpeedIntent{Speed: 210, Type: SpeedAtOrBelow},
			qualifiers: []string{"or less", "do not exceed", "not exceeding"},
		},
		{
			name:       "after fix or greater",
			intent:     SpeedIntent{Speed: 250, Type: SpeedAtOrAbove, AfterFix: "ROSLY"},
			qualifiers: []string{"or greater"},
		},
		{
			name:       "after fix or less",
			intent:     SpeedIntent{Speed: 210, Type: SpeedAtOrBelow, AfterFix: "ROSLY"},
			qualifiers: []string{"or less", "do not exceed"},
		},
		{
			name:       "until fix or greater",
			intent:     SpeedIntent{Speed: 250, Type: SpeedAtOrAbove, Until: &SpeedUntil{Fix: "ROSLY"}},
			qualifiers: []string{"or greater"},
		},
		{
			name:       "until fix or less",
			intent:     SpeedIntent{Speed: 210, Type: SpeedAtOrBelow, Until: &SpeedUntil{Fix: "ROSLY"}},
			qualifiers: []string{"or less"},
		},
	} {
		t.Run(test.name, func(t *testing.T) {
			for seed := uint64(1); seed <= 20; seed++ {
				readback := renderIntentForTest(test.intent, seed)
				assertContainsAny(t, readback, test.qualifiers...)
				if !strings.Contains(readback, "knots") {
					t.Fatalf("speed readback missing speed: %q", readback)
				}
				if test.intent.Until != nil && !strings.Contains(readback, "until") {
					t.Fatalf("speed readback missing until: %q", readback)
				}
			}
		})
	}
}

func TestCompoundSpeedReadbackIncludesQualifiers(t *testing.T) {
	above := MakeAtOrAboveSpeedRestriction(250)
	below := MakeAtOrBelowSpeedRestriction(210)
	intent := CompoundSpeedIntent{
		Segments: []CompoundSpeedSegment{
			{Speed: &above, UntilFix: "ROSLY"},
			{Speed: &below},
		},
	}

	for seed := uint64(1); seed <= 20; seed++ {
		readback := renderIntentForTest(intent, seed)
		assertContainsAny(t, readback, "or greater")
		assertContainsAny(t, readback, "or less")
		if !strings.Contains(readback, "rosly") {
			t.Fatalf("compound speed readback missing until fix: %q", readback)
		}
	}
}
